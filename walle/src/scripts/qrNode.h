//qrNode.h
//create a container to hold a QR code's global (xy) position and its message content
//created 2017-03-12
//////////////////////////////////////////////////////////////

#ifndef qrNode_h
#define qrNode_h

//#include <std::string.h>
//using namespace std;

struct qrNode
{
    double position_x;
    double position_y;

    std::string word;

    qrNode* next;
    //qrNode* prev;

    //qrNode(double x, double y, string code_word);
};

/*qrNode::qrNode(double x, double y, string code_word)
{
    position_x = x;
    position_y = y;
    word = code_word;

    next=NULL;*/

#endif
