# FOC firmware

Field Oriented Control firmware for boards using STM32G4 series mcus.

## Setup

Make sure the stm32-cmake submodule is initialised:

```
git submodule update --init --recursive
```

Select a build preset (defined in CMakePresets.json)

## Dependencies

```
CMake >= 4.0
Ninja
stm32-cmake
```

## Directories

```config/``` is for configuration headers of external libraries\
```middlewares/``` is for STM32 middlewares (e.g. USB)\
```src/``` is FOC firmware
