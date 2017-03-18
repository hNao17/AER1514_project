//qrLinkedList.h
//create linked list of qrNode structs
//created 2017-03-12
///////////////////////////////////////////////////////////////
#ifndef qrLinkedList_h
#define qrLinkedList_h

#include<iostream>
#include "qrNode.h"

class qrLinkedList
{
    private:
        qrNode* head;
        int listLength;

    public:
        qrLinkedList(); //constructor
        int getSize(); //determine # of qr codes in the list
        bool insertNode(qrNode* newNode); //insert new qr code into list queue
        bool searchList(std::string newQRWord); //search list for a specific qr code
        void printWordList();//print the qr code word list
};
#endif
