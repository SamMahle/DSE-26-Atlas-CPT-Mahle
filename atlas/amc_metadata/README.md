# Offline ArduPilot parameter documentation for AMC

ArduPilot Methodic Configurator reads `apm.pdef.xml` — ArduPilot's parameter
documentation for one firmware release — to label, range-check and convert
parameters. When a project folder does not contain it, AMC downloads it from
`autotest.ardupilot.org` the first time the project opens. An offline machine
cannot.

Put the files here, one folder per release, while a connected machine is
available:

```
amc_metadata/
    Copter-4.6.3/apm.pdef.xml
    Copter-4.7.0/apm.pdef.xml
    Plane-4.6.3/apm.pdef.xml
```

Download from
`https://autotest.ardupilot.org/Parameters/versioned/<Copter|Plane|Rover>/stable-<version>/apm.pdef.xml`,
or copy one from a project AMC has already opened online. The folder names
are for people; ATLAS reads the release from the line
`Generated from git tag Copter-4.6.3` inside each file.

On export, ATLAS copies the file whose release matches the build's firmware —
the exact release if present, otherwise the same major.minor — into the
project. If none matches, the export still works and the review lists it.
