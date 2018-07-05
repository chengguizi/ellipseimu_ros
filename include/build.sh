#!/bin/sh
# This script is used to build the sbgCom library on unix systems.
# To compile the library, you have to specify the byte ordering.
# Example: ./build.sh SBG_PLATFORM_LITTLE_ENDIAN

# Test that we have the endianness argument
if [ $# -ne 1 ]; then
    echo "You have to specify the platform endianness using either SBG_PLATFORM_BIG_ENDIAN or SBG_PLATFORM_LITTLE_ENDIAN"
    exit 1
fi

# Test the first argument and define the GCC options according to the selected endianness
if [ "$1" = "SBG_PLATFORM_BIG_ENDIAN" ]; then
    # The platform is in big endian
    gccOptions="-c -Wall -D SBG_PLATFORM_BIG_ENDIAN"
elif [ "$1" = "SBG_PLATFORM_LITTLE_ENDIAN" ]; then
    # The platform is in little endian
    gccOptions="-c -Wall -D SBG_PLATFORM_LITTLE_ENDIAN"
else
    echo "You have entered an invalid argument"
    exit 1
fi

# Create the intermediate directory		
mkdir obj

# Create all objects for binary logs directory
gcc $gccOptions binaryLogs/binaryLogDebug.c -o obj/binaryLogDebug.o
gcc $gccOptions binaryLogs/binaryLogDvl.c -o obj/binaryLogDvl.o
gcc $gccOptions binaryLogs/binaryLogEkf.c -o obj/binaryLogEkf.o
gcc $gccOptions binaryLogs/binaryLogEvent.c -o obj/binaryLogEvent.o
gcc $gccOptions binaryLogs/binaryLogGps.c -o obj/binaryLogGps.o
gcc $gccOptions binaryLogs/binaryLogImu.c -o obj/binaryLogImu.o
gcc $gccOptions binaryLogs/binaryLogMag.c -o obj/binaryLogMag.o
gcc $gccOptions binaryLogs/binaryLogOdometer.c -o obj/binaryLogOdometer.o
gcc $gccOptions binaryLogs/binaryLogPressure.c -o obj/binaryLogPressure.o
gcc $gccOptions binaryLogs/binaryLogs.c -o obj/binaryLogs.o
gcc $gccOptions binaryLogs/binaryLogShipMotion.c -o obj/binaryLogShipMotion.o
gcc $gccOptions binaryLogs/binaryLogStatus.c -o obj/binaryLogStatus.o
gcc $gccOptions binaryLogs/binaryLogUsbl.c -o obj/binaryLogUsbl.o
gcc $gccOptions binaryLogs/binaryLogUtc.c -o obj/binaryLogUtc.o

# Create all objects for commands directory
gcc $gccOptions commands/commandsAdvanced.c -o obj/commandsAdvanced.o
gcc $gccOptions commands/commandsCommon.c -o obj/commandsCommon.o
gcc $gccOptions commands/commandsEvent.c -o obj/commandsEvent.o
gcc $gccOptions commands/commandsGnss.c -o obj/commandsGnss.o
gcc $gccOptions commands/commandsInfo.c -o obj/commandsInfo.o
gcc $gccOptions commands/commandsInterface.c -o obj/commandsInterface.o
gcc $gccOptions commands/commandsMag.c -o obj/commandsMag.o
gcc $gccOptions commands/commandsOdo.c -o obj/commandsOdo.o
gcc $gccOptions commands/commandsOutput.c -o obj/commandsOutput.o
gcc $gccOptions commands/commandsSensor.c -o obj/commandsSensor.o
gcc $gccOptions commands/commandsSettings.c -o obj/commandsSettings.o


# Create all objects for interfaces directory
gcc $gccOptions interfaces/interfaceFile.c -o obj/interfaceFile.o
gcc $gccOptions interfaces/interfaceUdp.c -o obj/interfaceUdp.o
gcc $gccOptions interfaces/interfaceSerialUnix.c -o obj/interfaceSerialUnix.o

# Create all objects for misc directory
gcc $gccOptions misc/sbgCrc.c -o obj/sbgCrc.o
gcc $gccOptions misc/transfer.c -o obj/transfer.o

# Create all objects for protocol directory
gcc $gccOptions protocol/protocol.c -o obj/protocol.o

# Create all objects for time directory
gcc $gccOptions time/sbgTime.c -o obj/sbgTime.o

# Create all objets for the root directory
gcc $gccOptions sbgECom.c -o obj/sbgECom.o

# Create the library
ar cr ../lib/libSbgECom.a obj/binaryLogDebug.o obj/binaryLogDvl.o obj/binaryLogEkf.o obj/binaryLogEvent.o obj/binaryLogGps.o obj/binaryLogImu.o obj/binaryLogMag.o obj/binaryLogOdometer.o obj/binaryLogPressure.o obj/binaryLogs.o obj/binaryLogShipMotion.o obj/binaryLogStatus.o obj/binaryLogUsbl.o obj/binaryLogUtc.o obj/commandsAdvanced.o obj/commandsCommon.o obj/commandsEvent.o obj/commandsGnss.o obj/commandsInfo.o obj/commandsInterface.o obj/commandsMag.o obj/commandsOdo.o obj/commandsOutput.o obj/commandsSensor.o obj/commandsSettings.o obj/interfaceFile.o obj/interfaceUdp.o obj/interfaceSerialUnix.o obj/sbgCrc.o obj/transfer.o obj/protocol.o obj/sbgTime.o obj/sbgECom.o

rm -rf obj