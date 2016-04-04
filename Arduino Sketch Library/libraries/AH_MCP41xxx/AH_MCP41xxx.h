
/*************************************************************************
**  Device: MCP41010/MCP41050/MCP42010/MCP42050                       	**
**  File:   AH_MCP41xxx.h 					    	**
**			  		     				**
**  Created by A. Hinkel 2012-09-30                                 	**
**  downloaded from http://www.arduino-projekte.de			**
**									**
**									**
**  Released into the public domain.  		                    	**
**                                                                  	**
*************************************************************************/


#ifndef AH_MCP41xxx_h
#define AH_MCP41xxx_h

#include "Arduino.h"

class AH_MCP41xxx
{
  public:
  //  AH_MCP41xxx();
    void init_MCP41xxx (int CS);    
    void init_MCP42xxx (int CS, int SHDN, int RS);

    void setValue (byte value);  //0-255
    void setValue (byte value, int potentiometer); 

    void shutdown (boolean data);
    void reset ();

  private:
    int  _CS;
    int  _SHDN;
    int  _RS;
    byte spi_transfer(byte data);
};

#endif
