#pragma once

#ifndef CreateTangoCmd_H_
#define CreateTangoCmd_H_

#include <maya/MPxCommand.h>
#include <string>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MSelectionList.h>
#include <maya/MDGModifier.h>
#include <maya/MPlug.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MStringArray.h>
#include <maya/MObjectArray.h>
#include <sstream>

class TangoCmd : public MPxCommand
{
public:
    //TangoNode t;
    TangoCmd();
    virtual ~TangoCmd();
    static void* creator();
    MStatus doIt(const MArgList& args);
    
    static const MTypeId kNODE_ID;
    static const char* kIN_TRANSFORM_ATTR_NAME;
    static const char* kMSG_CXN_ATTR_NAME;

    // for node creation
    MStatus redoIt();
    MStatus undoIt();
    bool isUndoable() const;
    static MSyntax newSyntax();
    /// The name of the command that is meant to be run.
    static const MString kCOMMAND_NAME;
    MStatus parseArgs(const MArgList& args);

    /// Storage for the flag arguments that will be passed into the command.
    bool flagHelpSpecified = false;
    MStringArray flagSelList;

    /// Storage for the operations that this command performs on the DG so that we
    /// can undo them if necessary.
    MDGModifier dgMod;
};

#endif