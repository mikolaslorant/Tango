#include "TangoCmd.h"
#include <maya/MGlobal.h>
#include <list>

const MTypeId TangoCmd::kNODE_ID(0x0);
const MString TangoCmd::kCOMMAND_NAME = "TangoCmd";
const char* TangoCmd::kIN_TRANSFORM_ATTR_NAME = "inTransform";
const char* TangoCmd::kMSG_CXN_ATTR_NAME = "locatorCallBackAttr";
TangoCmd::TangoCmd() : MPxCommand()
{
}

TangoCmd::~TangoCmd()
{
}

const char* flagSelListLongName = "-node";
const char* flagSelListShortName = "-n";

const char* flagHelpLongName = "-help";
const char* flagHelpShortName = "-h";

const char* helpText = "This command will setup a callback on a given node.\n"
"Usage:\n   applyCallback [options]\n"
"Options:\n"
"-h / -help     Prints this message.\n\n"
"-n / -node     The name of the node to setup the callback example for.\n\n";


void* TangoCmd::creator()
{
    return new TangoCmd();
}

MSyntax TangoCmd::newSyntax()
{
    MSyntax syntax;
    syntax.addFlag(flagHelpShortName, flagHelpLongName);
    syntax.addFlag(flagSelListShortName, flagSelListLongName, MSyntax::kSelectionItem);
    syntax.enableQuery(false);
    syntax.enableEdit(false);
    syntax.useSelectionAsDefault(true);

    return syntax;
}


MStatus TangoCmd::parseArgs(const MArgList& args)
{
    MStatus result;
    MArgDatabase argDb(syntax(), args, &result);
    CHECK_MSTATUS_AND_RETURN_IT(result);

    if (argDb.isFlagSet(flagHelpShortName)) {
        displayInfo(helpText);
        flagHelpSpecified = true;
        return MStatus::kSuccess;
    }
    else {
        flagHelpSpecified = false;
    }

    if (argDb.isFlagSet(flagSelListShortName)) {
        argDb.getFlagArgument(flagSelListShortName, 0, flagSelList);
    }

    return result;
}


MStatus TangoCmd::doIt(const MArgList& args)
{
	// message in Maya output window
	// cout<<"Implement Me!"<<endl;
	//std::cout.flush();

	// message in scriptor editor
	//MGlobal::displayInfo("Implement Me!");	
	//return MStatus::kSuccess;

    setCommandString(TangoCmd::kCOMMAND_NAME);
    clearResult();

    MStatus result = parseArgs(args);
    CHECK_MSTATUS_AND_RETURN_IT(result);

    if (this->flagHelpSpecified == true) {
        return MStatus::kSuccess;
    }

    return redoIt();
}

MStatus TangoCmd::redoIt()
{
    MStatus result;
    if (flagSelList.length() != 1) {
        MGlobal::displayError("You need to select a single node to apply the callback to!");
        return MStatus::kInvalidParameter;
    }

    //if (doesTangoNodeAlreadyExist() == true) {
    //    MGlobal::displayError("The feature already exists!");
    //    return MStatus::kFailure;
    //}

    MObject callbackNode = dgMod.createNode("TangoNode", &result);
    CHECK_MSTATUS_AND_RETURN_IT(result);
    result = dgMod.doIt();
    CHECK_MSTATUS_AND_RETURN_IT(result);

    MFnDependencyNode fnNode(callbackNode);
    MPlug callbackNodeMsgPlug = fnNode.findPlug("inTransform",
        false,
        &result);
    CHECK_MSTATUS_AND_RETURN_IT(result);

    MObject transform;
    result = flagSelList.getDependNode(0, transform);   // gets transform of first element of the selection list
    CHECK_MSTATUS_AND_RETURN_IT(result);
    if (!transform.hasFn(MFn::kDependencyNode)) {
        MGlobal::displayError("The object specified is not a valid DG node!");
        return MStatus::kInvalidParameter;
    }
    result = fnNode.setObject(transform);
    CHECK_MSTATUS_AND_RETURN_IT(result);

    if (!fnNode.hasAttribute("locatorCallBackAttr")) {  // checks if transform has callback attribute, if not present, create that callback attribute
        MFnMessageAttribute fnMsgAttr;
        MObject msgAttr = fnMsgAttr.create("locatorCallBackAttr",
            "locatorCallBackAttr",
            &result);
        CHECK_MSTATUS_AND_RETURN_IT(result);
        fnNode.addAttribute(msgAttr);   // add callback attribute to locator transform
    }
    MDGModifier dgModCxn;   // dgmodifier for callback operations
    MPlug msgPlug = fnNode.findPlug("locatorCallBackAttr", false, &result);
    CHECK_MSTATUS_AND_RETURN_IT(result);
    result = dgModCxn.connect(msgPlug, callbackNodeMsgPlug);    // connects callback attributes of TangoNode and the locator transform node
    CHECK_MSTATUS_AND_RETURN_IT(result);
    dgModCxn.doIt();
    return result;
}

MStatus TangoCmd::undoIt()
{
    dgMod.undoIt();
    return MStatus::kSuccess;
}


bool TangoCmd::isUndoable() const
{
    if (flagHelpSpecified == true) {
        return false;
    }
    else {
        return true;
    }
}