# Mechanism Change PR: [Mechanism Name]

## Description
<!-- Provide a clear description of the mechanism changes -->

## Safety Checks
- [ ] Motor current limits properly configured
- [ ] Software limits implemented and tested
- [ ] Mechanism collision prevention validated

## Code Quality
- [ ] Code follows team style guide
- [ ] Comments added for complex logic
- [ ] Constants defined in appropriate subsystem constants file
- [ ] No magic numbers in code

## Logging
- [ ] Full AdvantageKit logging implemented for all subsystem states
- [ ] Replay support verified with AdvantageScope
- [ ] Custom AdvantageScope visualizations added if needed

## Robot Testing
- [ ] Tested on competition bot
- [ ] Tested with full battery (>12V)
- [ ] Tested with 11.8V battery (if possible)
- [ ] Tested while other mechanisms running
- [ ] Tested in all robot positions/configurations

## Competition Readiness
- [ ] Compatible with all auto modes
- [ ] Works with current operator controls or has been updated
- [ ] Dashboard widgets updated if needed
- [ ] Fail-safe behaviors implemented
- [ ] Recovery procedures documented

## Leadership Verification
- [ ] Team leadership has tested/verified functionality

## Pre-merge Checklist
- [ ] Code reviewed by leader
- [ ] (Optional) Changes tested across multiple matches
- [ ] No merge conflicts with production
- [ ] CI/CD pipeline passing
- [ ] Branch up to date with production

## Documentation
- [ ] Full Javadoc comments on all classes and variables
- [ ] Complete mkdocs documentation added/updated
- [ ] Update driver documentation
- [ ] Update prematch checklist if needed
- [ ] Update mechanism troubleshooting guide
- [ ] Notify relevant team members
- [ ] Schedule driver practice with changes