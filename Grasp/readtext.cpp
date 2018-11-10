// file read library
// originally coded by Jin Yamanaka
//----------------------------------------------------------------------
// $Log: readtext.cpp,v $
// Revision 1.1.1.1  2003/05/01 01:11:40  kanehiro
// OpenHRP sources
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

#include "readtext.h"

#include <iostream>
#include <string.h>

using namespace std;

namespace grasp{

// a line starts with "#" will be skipped.
bool
GetString( istream &in , char str[])
{
	char c;
	int i;
	
	in.get(c);
	while( c == ' ' || c == '\t' || c== 0x0D || c == 0x0A || c == '#' ){
		if( !in )return false;
		if( c == '#')
			while( c != 0x0A) in.get(c);
		in.get(c);
	}
	
	i=0;
	while( c != ' ' && c != '\t' && c != 0x0D && c != 0x0A){
		str[i++] = c;
		in.get(c);
		if( !in ){
			str[i] = 0;
			return false;
		}
	}
	str[i] = 0;
	
        in.putback(c);

        return true;
}

// get all sort of string 
bool
GetAllString2( istream &in , char str[])
{
	char c;
	int i;
	
	in.get(c);
	while( c == ' ' || c == '\t' || c== 0x0D || c == 0x0A){
		if( !in )return false;
		in.get(c);
	}
	
	i=0;
	while( c != ' ' && c != '\t' && c != 0x0D && c != 0x0A){
		str[i++] = c;
		in.get(c);
		if( !in ){
			str[i] = 0;
			return false;
		}
	}
	str[i] = 0;
	
        in.putback(c);

        return true;
}

void
BackString( istream &in , char str[])
{
	int i;
        for(i = strlen(str)-1;i >= 0;i--){
                in.putback(str[i]);
        }
}

void
SkipToEOL( istream &in )
{
        char c;
        in.get(c);
        while( c != 0x0A && in) in.get(c); 
}

bool
GetDouble( istream &in, double &data)
{
	char c;
	
	in.get(c);
	while( c == ' ' || c == '\t' || c== 0x0D || c == 0x0A || c == '#' ){
		if( !in ) return false;
		if( c == '#')
			while( c != 0x0A) in.get(c);
		in.get(c);
	}

	in.putback(c);

        in >> data;

        return true;
}

bool
GetInteger( istream &in, int &data)
{
	char c;
	
	in.get(c);
	while( c == ' ' || c == '\t' || c== 0x0D || c == 0x0A || c == '#' ){
		if( !in ) return false;
		if( c == '#')
			while( c != 0x0A) in.get(c);
		in.get(c);
	}

	in.putback(c);

        in >> data;

        return true;
}

// '03 Jan.17  when it reaches the end of line, return false 
// '07 Jan.24  putback control character   
bool
GetLineDouble( istream &in, double &data)
{
	char c;
	
        if( !in ) return false;
	in.get(c);
	while( c == ' ' || c == '\t' || c== 0x0D || c == 0x0A || c == '#' ){
		if( !in || c == 0x0A ){
                        in.putback(c);
                        return false;
                }
		if( c == '#')
			while( c != 0x0A) in.get(c);
                else
                        in.get(c);
	}

	in.putback(c);

        in >> data;

        return true;
}

// for VRML coordinate data
// 現在のファイルポインタから最初の数字データを発見する。
// ']' が見付かったらfalseを返す
bool
GetVRMLdouble( istream &in, double &data, bool verbose)
{
	char c;
	
	in.get(c);
        if( c == ']') return false; // point data end

        while( !(isdigit(c) || c == '+' || c == '-' || c == '.') ){
                if(verbose) cout << c;
		if(  !in    ) return false;  			// EOF
                if( c == ']') return false; 			// data field end
		if( c == '#') while( c != 0x0A) in.get(c);  	// skip comment
		in.get(c);
	}
	in.putback(c);

        in >> data;

        return true;
}



}
