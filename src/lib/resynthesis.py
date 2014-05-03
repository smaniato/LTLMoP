import itertools
import project
import re
import fsa
import parseSpec
from LTLParser.LTLFormula import LTLFormula, LTLFormulaType
from createJTLVinput import createLTLfile
import specCompiler

import threading
import logging

class ExecutorResynthesisExtensions(object):
    """ Extensions to Executor to allow for specification rewriting and resynthesis.
        This class is not meant to be instantiated. """

    def __init__(self):
        super(ExecutorResynthesisExtensions, self).__init__()
        logging.info("Initializing Executor Resynthesis")
        
        self.next_proj = None
        self.needs_resynthesis = False
        
        ####################################################################################
        # HACK: Wrap the fsa runIteration function to check for internal flags at each step.
        # (Alternatively, this could be done with handler mapping manipulations or we could
        # put conditional code inside executor but this is least intrusive for now)
        ####################################################################################
        
        original_runIteration = self.runStrategyIteration
        check_flags = self._checkForNewInternalFlags
        def runIterWithResynthesisChecks():
            original_runIteration()
            check_flags()
        self.runStrategyIteration = runIterWithResynthesisChecks
        
        self.user_query_response = ""
        self.received_user_query_response = threading.Event()
    
    def _checkForNewInternalFlags(self):

        if self.internalTriggers:
            for p in self.internalTriggers:
                self._processInternalFlag(p)
            self.internalTriggers = []
       
        ### See if the resynthesis actuator handler has let us know we need to resynthesize ###

        if self.needs_resynthesis:
            if self.next_proj is not None:
                pass
                #self.resynthesizeFromProject(self.next_proj)
            else:
                logging.error("Resynthesis was triggered before any spec rewrites.  Skipping.")

            # Clear the resynthesis flag
            self.needs_resynthesis = False
        
        
    def _processInternalFlag(self, flag_name):
        """ Respond appropriately to an "internal flag" proposition having been triggered.
                    Note that this is only called on rising edges. """

        #################################################
        ### Check for group modification propositions ###
        #################################################

        # Use a regex on the proposition name to figure out what we should do
        m = re.match(r"_(?P<action>add_to|remove_from)_(?P<groupName>\w+)", \
                     flag_name, re.IGNORECASE)
            
        # We currently only handle this one type of flag, so there's nothing to do
        # if it doesn't match
        if m is None:
            return
            
        if self.next_proj is None:
            self.next_proj = self._duplicateProject(self.proj)
            
        logging.info("Internal Flag Triggered")
        
        newPropName = self.hsub.prop2func[m.group('groupName')]()
        logging.info('\t Adding to group %s: %s', m.group('groupName'), newPropName)
        self._updateSpecGroup(m.group('groupName'), 'add', [newPropName])
        
        
        for k, v in self.hsub.prop2func.iteritems():
            pref = m.group('groupName')+'->'
            if(k.startswith(pref)):
                nextGroup = k.replace(pref, "")
                nextGroup_prop = self.hsub.prop2func[k]()
                logging.info('\t Adding to group %s: %s', nextGroup, nextGroup_prop)
                self._updateSpecGroup(nextGroup, 'add', [nextGroup_prop])
        if self.needs_resynthesis:
            logging.info("\t*Need Resynthesis")
        logging.info('----')
    
  
    
    def _updateSpecGroup(self, group_name, operator, operand):
            """ Rewrite the text of the specification in `self.next_proj` by modifying proposition
                group `group_name` with operator `operator` (e.g. "add"/"remove") and
                operand `operand` (a list of propositions). Rewrite .spec file of 'self.next_proj' """

            # Make a regex for finding the group definition in the spec
            PropositionGroupingRE = re.compile(r"^group\s+%s\s+(is|are)\s+(?P<propositions>.+)\n" % group_name, \
                                          re.IGNORECASE|re.MULTILINE)

            # Define a function for modifying the list of propositions in-place
            def gen_replacement_text(group_name, operand, m):
                """ Given a group name, modification operand, and match object from
                    PropositionGroupingRE, return a new group definition line with an appropriately
                    updated list of propositions."""

                # Figure out what propositions are already there
                propositions = re.split(r"\s*,\s*", m.group('propositions'))

                # Remove the "empty" placeholder if it exists
                propositions = [p for p in propositions if p != "empty"]

                # Perform the operation on the list of propositions
                if operator == "add":
                    propositions.extend(operand)
                elif operator == "remove":
                    propositions = [p for p in propositions if p not in operand]
                else:
                    logging.error("Unknown group modification operator {!r}".format(operator))

                # Re-add the "empty" placeholder if the result of the operation is now empty
                if not propositions:
                    propositions = ["empty"]

                # Create a new group definition line
                new_group_definition = "group %s is %s\n" % (group_name, ", ".join(propositions))

                return new_group_definition

            self.next_proj.specText = PropositionGroupingRE.sub(lambda m: gen_replacement_text(group_name, operand, m), \
                                                                self.next_proj.specText)
            logging.info("Updating next project's specText")
            self.next_proj.writeSpecFile(self.next_proj.getFilenamePrefix() + ".spec")
                                                                
                                                                
    def _duplicateProject(self, proj, n=itertools.count(1)):
        """ Creates a copy of a proj, and creates an accompanying spec file with an 
            auto-incremented counter in the name."""

        # reload from file instead of deepcopy because hsub stuff can include uncopyable thread locks, etc
        new_proj = project.Project()
        new_proj.setSilent(True)
        new_proj.loadProject(self.proj.getFilenamePrefix() + ".spec")

        # copy hsub references manually
        new_proj.h_instance = proj.h_instance

        # Choose a name by incrementing the stepX suffix
        # Note: old files from previous executions will be overwritten
        base_name = self.proj.getFilenamePrefix().rsplit('.',1)[0] # without the modifier
        newSpecName = "%s.step%d.spec" % (base_name, n.next())

        new_proj.writeSpecFile(newSpecName)

        logging.info("Wrote new spec file: %s" % newSpecName)
        
        return new_proj

    def _setSpecificationInitialConditionsToCurrent(self, proj):
        """ Remove any existing initial conditions from the guarantees portion of the LTL specification
            and replace them with the current state of the system.

            Propositions that don't exist in both old and new specifications are ignored in the process."""

        # TODO: support doing this at the language level too?
        # TODO: what if state changes during resynthesis? should we be less restrictive?

        # parse the spec so we can manipulate it
        ltl_filename = proj.getFilenamePrefix() + ".ltl"
        assumptions, guarantees = LTLFormula.fromLTLFile(ltl_filename)

        # TODO: do we need to remove too? what about env?
        # add in current system state to make strategy smaller
        ltl_current_state = self.getCurrentStateAsLTL() # TODO: constrain to props in new spec
        gc = guarantees.getConjuncts()

        if ltl_current_state != "":
            gc.append(LTLFormula.fromString(ltl_current_state))

        # write the file back
        createLTLfile(ltl_filename, assumptions, gc)

    def resynthesizeFromNewSpecification(self, spec_text):
        self.pause()

        self.postEvent("INFO", "Starting resynthesis...")

        # Copy the current project
        new_proj = self._duplicateProject(self.proj)

        # Overwrite the specification text
        new_proj.specText = spec_text

        # Save the file
        new_proj.writeSpecFile()

        # Get a SpecCompiler ready
        c = specCompiler.SpecCompiler()
        c.proj = new_proj

        # Make sure rfi is non-decomposed here
        c.proj.loadRegionFile(decomposed=False)

        if c.proj.compile_options["decompose"]:
            c._decompose()

        # Call the parser
        c._writeLTLFile()
        c._writeSMVFile()

        # Constrain the initial conditions to our current state
        self._setSpecificationInitialConditionsToCurrent(new_proj)

        # Synthesize a strategy
        (realizable, realizableFS, output) = c._synthesize()
        logging.debug(output)

        if not (realizable or realizableFS):
            logging.error("Specification for resynthesis was unsynthesizable!")
            self.pause()
            return False

        logging.info("New automaton has been created.")

        # Load in the new strategy

        self.proj = new_proj

        logging.info("Reinitializing execution...")

        spec_file = self.proj.getFilenamePrefix() + ".spec"
        aut_file = self.proj.getFilenamePrefix() + ".aut"
        self.initialize(spec_file, aut_file, firstRun=False)

        self.resume()

        return True
        
        # TODO: reload from file less often
    
    def processUserQueryResponse(self, answer):
        """ Callback function to receive a response to a user query. """

        logging.debug("Got user query response {!r}".format(answer))

        # Save the response
        self.user_query_response = answer

        # Let anyone waiting for the response know that it is ready
        self.received_user_query_response.set()

        
    



