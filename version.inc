# Included by CMakeLists, edited by the build script
# (the version numbers are generated by the build script from the git current label)

# name of the slicer
set(SLIC3R_APP_NAME "NematXSlicer")
# Key fo the slicer, must only contains ascii chars and no spaces (be valid in all filesystems)
set(SLIC3R_APP_KEY "NematXSlicer")
# exe name of the slicer, should be lowercase and valid in all os
set(SLIC3R_APP_CMD "nematxslicer")
# id for windows registry
set(SLIC3R_APP_PROG_ID "Slic3r.1")
# versions
set(SLIC3R_VERSION "2.7+UNKNOWN")
set(SLIC3R_VERSION_FULL "2.7.61.0")
set(SLIC3R_BUILD_ID "${SLIC3R_APP_KEY}_${SLIC3R_VERSION_FULL}+UNKNOWN")
set(SLIC3R_RC_VERSION "2,7,61,0")
set(SLIC3R_RC_VERSION_DOTS "${SLIC3R_VERSION_FULL}")

# Same as the slicer name but for gcodeviewer
set(GCODEVIEWER_APP_NAME "${SLIC3R_APP_NAME} G-code Viewer")
set(GCODEVIEWER_APP_KEY "${SLIC3R_APP_KEY}-gcodeviewer")
set(GCODEVIEWER_APP_CMD "${SLIC3R_APP_CMD}-gcodeviewer")
set(GCODEVIEWER_APP_PROG_ID "${SLIC3R_APP_NAME}.GCodeViewer.1")

# string to be added after the SLIC3R_APP_NAME on some places
set(SLIC3R_BASED_ON "")
# to get he github repo
set(SLIC3R_GITHUB "NematX/NematXSlicer")
# download url
set(SLIC3R_DOWNLOAD "https://github.com/NematX/NematXSlicer/actions")
set(SLIC3R_DOC_URL "https://github.com/NematX/NematXSlicer/wiki")
