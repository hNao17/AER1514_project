//qrCode.h
//create a container to hold a QR code's global (xy) position and its message content
//created 2017-04-05
//////////////////////////////////////////////////////////////

#ifndef qrCode_h
#define qrCode_h

//#include <std::string.h>
//using namespace std;

struct qrCode
{
    double position_x;
    double position_y;
    double angle;

    std::string word;

};

#endif
