#ifndef CreateTangoCmd_H_
#define CreateTangoCmd_H_

#include <maya/MPxCommand.h>
#include <string>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <sstream>

class TangoCmd : public MPxCommand
{
public:
    //TangoNode t;
    TangoCmd();
    virtual ~TangoCmd();
    static void* creator() { return new TangoCmd(); }
    MStatus doIt(const MArgList& args);
};

#endif