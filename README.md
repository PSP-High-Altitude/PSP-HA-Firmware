# README #

### What is this repository for? ###
This repository contains all the firmware for the SRAD flight computer being designed by PSP High Altitude. It contains a [PlatformIO](https://platformio.org/) project that is intended to be used with VS Code and the PlatformIO IDE extension.

This codebase is intended to be platform agnostic, and though it may contain drivers for specific hardware, the general goal is to create a modular and reusable set of libraries that can be reused in case of hardware or project changes.

### How do I get set up? ###
Follow the [installation instructions](https://platformio.org/install/ide?install=vscode) on the PlatformIO website to install VS Code (if you don't already have it) and the PlatformIO IDE extension. Once you have it installed, clone this repository to your machine and open it in VS Code:
```
git clone https://github.com/PSP-High-Altitude/PSP-HA-Firmware.git
cd PSP-HA-Firmware
code .
```
Once you are in VS Code, you should see various PlatformIO buttons on the blue bar at the bottom. Click the check mark icon on the toolbar to start building the project:

<img width="448" alt="image" src="https://user-images.githubusercontent.com/34579088/198095014-0b220710-ba9a-40e1-8d4e-2d361cc9c395.png">

The first build may take a while as PlatformIO needs to install all the necessary dependencies and toolchains, so be patient.

You can find a lot of resources and documentation about the firmware as well as surrounding avionics components in our [Confluence](https://purdue-space-program.atlassian.net/wiki/spaces/SASD/overview).

### Contribution guidelines ###
#### Code Style ####
Having consistent code style and formatting is imperative for a project which is intended to be worked on by many people over a long time. Code formatting is enforced by clang-format, and if you are using VS Code in this repository, it should automatically auto format each file on save. If you are using a different editor or if auto format is not working on your machine, please make sure that you manually run clang-format on each file you edited before committing them.

For other aspects of code style such as naming, we are not religiously sticking to any particular existing scheme, but it is important that consistency is maintained. As a general rule, variables and functions should be `lower_case`, data types such as structs and enum types should be `PascalCase`, and defines and enum values should be `UPPER_CASE`. When in doubt, look at existing code and try to mimic it.

#### Tests ####
Whenever possible, any new components should also add unit tests for verifying the functionality and correctness of the code. For a primer on how unit tests are set up, reference the corresponding [PlatformIO unit testing documentation](https://docs.platformio.org/en/stable/advanced/unit-testing/index.html). To ensure both ease of testing, tests must exist for the native platform, and optionally for any embedded platforms the components are intended to be used on.

Sometimes it may be desirable to write testing code that runs as the main program on a microcontroller instead of running within one of the unit testing frameworks. In cases like this, make sure that any testing code you add to `src` does not get committed. `src` should only contain the working version of the flight software. If you want to preserve any testing code, commit it to a different experiments repository.
