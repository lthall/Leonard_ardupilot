# Configuration Profiles LUA script

This script allows different configuration profiles to be selected by changing single parameters created by the script.

You may use this script to allow users to rapidly reconfigure the vehicle based on the role the vehicle will be used in.

## Terms

### Domains

Parameters are divided into domains; a parameter can only exist in a single domain.  So you may have a domain for different battery configurations and a domain for different payload configurations.

Each domain specifies what parameters are relevant in all_param_defaults attribute.

### Profiles
Each domain has a number of profiles which can be switched between through setiting a parameter.  The profiles specify parameter values which override the defaults specified in the domain all_param_defaults attribute.

### Modes

Each domain can be in one of three modes:
 - 0: do nothing
 - 1: use parameters from the selected profile, taking the parameter value from the defaults stored in the domain if the profile does not specify a value
 - 2: use the default parameter values only

### Use

Switch between modes using the "_MODE" parameter, and between selections with the "_SEL" parameter.

So if you have a domain "ARM" then you might set CFG_PAY_MODE to 1 to use the parameters from the profile and CFG_PAY_SEL to 2 to select the second payload configuration.

Any time a parameter is changed by the script a reboot is required.
