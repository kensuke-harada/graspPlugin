// text file read library
// originally coded by Jin Yamanaka
//----------------------------------------------------------------------
// $Log: readtext.h,v $
// Revision 1.2  2004/04/22 09:20:01  kanehiro
// update support of win32
//
// Revision 1.1.1.1  2003/05/01 01:11:40  kanehiro
// OpenHRP sources
//
// Revision 1.7  2003/04/25 12:09:55  kanehiro
// update win32 support
//
// Revision 1.6  2003/03/24 07:48:21  kajita
// adapt to Windows: GetAllString() => GetAllString2()
//
// Revision 1.5  2003/03/04 02:09:29  kanehiro
// ifstream -> istream
//
// Revision 1.4  2003/01/17 09:16:41  kajita
// GetLineDouble
//
//----------------------------------------------------------------------

#ifndef _READTEXT_H
#define _READTEXT_H
//#include <iostream>
#include <fstream>
#include <string>


#ifndef DllExport
#ifdef _WIN32
#define DllExport __declspec(dllexport)
#else
#if __GNUC__ >= 4
#define DllExport __attribute__ ((visibility("default")))
#else
#define DllExport 
#endif
#endif
#endif


//#include "exportdef.h"

namespace grasp{


bool DllExport GetString( std::istream &in , char str[]);
bool DllExport GetAllString2( std::istream &in , char str[]);
void DllExport BackString( std::istream &in , char str[]);
void DllExport SkipToEOL( std::istream &in );
bool DllExport GetDouble( std::istream &in, double &data);
bool DllExport GetInteger( std::istream &in, int &data);

bool DllExport GetLineDouble( std::istream &in, double &data);
bool DllExport GetVRMLdouble( std::istream &in, double &data, bool verbose=false);
	
	
}

#endif
