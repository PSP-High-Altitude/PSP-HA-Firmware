Contributing
============

The PSP-HA-Firmware is one of the main ongoing Avionics projects and members are expected and encouraged to
contribute. This being said, certain guidelines should be followed when contributing to code and documentation,

Code Style
----------

Having consistent code style and formatting is imperative for a project which is intended to be worked on 
by many people over a long time. Code formatting is enforced by clang-format, and if you are using VS Code 
in this repository, it should automatically auto format each file on save. If you are using a different 
editor or if auto format is not working on your machine, please make sure that you manually run clang-format 
on each file you edited before committing them.

For other aspects of code style such as naming, we are not religiously sticking to any particular existing 
scheme, but it is important that consistency is maintained. As a general rule, variables and functions should 
be lower_case, data types such as structs and enum types should be PascalCase, and defines and enum values 
should be UPPER_CASE. When in doubt, look at existing code and try to mimic it.

Tests
-----

Whenever possible, any new components should also add unit tests for verifying the functionality and 
correctness of the code. For a primer on how unit tests are set up, reference the corresponding `PlatformIO 
unit testing documentation <https://docs.platformio.org/en/stable/advanced/unit-testing/index.html>`_. To ensure both ease of testing, tests must exist for the native platform, and 
optionally for any embedded platforms the components are intended to be used on.

Sometimes it may be desirable to write testing code that runs as the main program on a microcontroller 
instead of running within one of the unit testing frameworks. In cases like this, make sure that any testing 
code you add to src does not get committed. src should only contain the working version of the flight software. 
If you want to preserve any testing code, commit it to a different experiments repository.

Documentation
-------------

Documentation ensures that new members to come as well as people unfamilar with all parts of the project have
a comprehensive, up-to-date source of information. Whenever changes are made to the code, check all potentially
affected sections of documentation and update them accordingly. 

When deciding whether information is relevant enough to be documented, consider whether it is obvious to someone 
reading the code or if it may be potentially confusing. Document any potential constraints or "gotchas" to avoid 
problems down the road. If you are unsure of what should be documented, look at existing documentation and/or 
ask a more senior member of the project.