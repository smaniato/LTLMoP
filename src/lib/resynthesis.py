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
    
    def _checkForNewInternalFlags(self):

        if self.internalTriggers:
            # msg = "We have internal flags: "
            # for flag in self.internalTriggers:
            #     msg += flag + " "
            # logging.info(msg)
            for p in self.internalTriggers:
                self._processInternalFlag(p)
            self.internalTriggers = []
        if self.needs_resynthesis:
            logging.info("Need Resynthesis")
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
        
        # Create next_proj if we haven't yet
        # This is what we'll be working on, and then eventually resynthesizing from
        if self.next_proj is None:
            d=3
            #logging.info("Need to duplicate project")
            #self.next_proj = self._duplicateProject(self.proj)

        # We've been told to add or remove something from a group, but we need to figure
        # out exactly what that /something/ is (the "referent").
        response = "new" + flag_name
        
        referents = [response]
        
        for ref in referents:
            if m.group('action').lower() == "add_to":
                logging.info("Added item(s) %s to group %s.", ", ".join(referents), m.group('groupName'))
                
                #Rewrite the group definition in the specification text
                #self._updateSpecGroup(m.group('groupName'), 'add', referents)
                
                #Get a list of 
                # Figure out if there are any corresponding groups which we will also need to update
                corresponding_groups = parseSpec._findGroupsInCorrespondenceWithGroup(self.proj, m.group('groupName'))
                logging.info("Need to also update corresponding groups: {}".format(corresponding_groups))
                
            
    
    def _updateSpecGroup(self, group_name, operator, operand):
            """ Rewrite the text of the specification in `self.next_proj` by modifying proposition
                group `group_name` with operator `operator` (e.g. "add"/"remove") and
                operand `operand` (a list of propositions) """

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
                                                                
                                                                
    def _duplicateProject(self, proj, n=itertools.count(1)):
        """ Creates a copy of a proj, and creates an accompanying spec file with an 
            auto-incremented counter in the name."""

        # reload from file instead of deepcopy because hsub stuff can include uncopyable thread locks, etc
        new_proj = project.Project()
        new_proj.setSilent(True)
        new_proj.loadProject(self.proj.getFilenamePrefix() + ".spec")

        # copy hsub references manually
        new_proj.hsub = proj.hsub
        new_proj.hsub.proj = new_proj # oh my god, guys
        new_proj.h_instance = proj.h_instance
        
        new_proj.sensor_handler = proj.sensor_handler
        new_proj.actuator_handler = proj.actuator_handler

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




