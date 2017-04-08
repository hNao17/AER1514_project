//qrLinkedList.cpp
//contains qrLinkedList class definitions
//created on 2017-03-20
///////////////////////////////////////////////////
#include<ros/ros.h>
#include "qrLinkedList.h"

qrLinkedList::qrLinkedList()
{
    //head = new qrNode;
    listLength = 0;

    head->next=NULL;
    //qrNode->prev=NULL;
    //head->word = "qr list does not contain any qr codes";
    head->position_x = -1;
    head->position_y = -1;

    ROS_INFO_STREAM("Successfully created null qr head node");
    ROS_INFO_STREAM("List length = "<<listLength);
}

int qrLinkedList::getSize()
{
    return listLength;
}

bool qrLinkedList::insertNode(qrNode* newNode)
{
    //new node is always inserted in a FIFO (queue) manner

    //list is empty; new node is inserted at the head position
    if(listLength==0)
    {
        head = newNode;
        ROS_INFO_STREAM("Successfully inserted qr node at head position");
        listLength++;
        return true;
    }

    qrNode* index=head;
    qrNode* temp=index;

    //list is not empty
    while(index)
    {
        temp = index;
        //find end position of list
        if(temp->next==NULL)
        {
            //add new node to the next position in the list
            temp->next = newNode;
            newNode->next = NULL;
            ROS_INFO_STREAM("Sucessfully inserted "<<temp->word<<" at end of list");
            listLength++;
            return true;
        }

        //update index to look at next node in the list
        index=temp->next;
    }

    ROS_WARN_STREAM("Unable to add qr code to list");
    return false;

}

bool qrLinkedList::searchList(std::string newQRWord)
{
    qrNode* index=head;
    qrNode* temp=index;

    if(listLength <=0)
    {
        ROS_WARN_STREAM("List is empty");
        return false;
    }

    //search list until qrNode->next == NULL
    while(index)
    {
        temp = index;

        //determine if qr word is already in the list
        if(temp->word == newQRWord)
        {
            ROS_WARN_STREAM(newQRWord<<" already exists in the list.  Do not insert qr node.");
            return true;
        }

        //update index to look at the next node in the list
        index=temp->next;
    }

    ROS_INFO_STREAM("Word not found.  Insert qr code into list");
    return false;
}

void qrLinkedList::printWordList()
{
    if(listLength <=0)
    {
        ROS_WARN_STREAM("List is empty, cannot print list");
        return;
    }

    qrNode* index=head;
    qrNode* temp=head;

    int count=0;

    //search list until qrNode->next == NULL
    while(index)
    {
        count++;
        temp = index;

        //print current word in the list
        ROS_INFO_STREAM("Word" <<count<<" =\t"<<temp->word);

        //update index to look at the next node in the list
        index=temp->next;
    }

}
