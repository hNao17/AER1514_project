#include <ros/ros.h>
#include "../qrLinkedList.h"
#include "../qrLinkedList.cpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "supervisor_linkedList_test");
	ros::NodeHandle nh;

	qrNode* a=new qrNode;

	a->position_x=10;
	a->position_y=10;
	a->word="Kasper";
	a->next=NULL;

    qrNode* b=new qrNode;

	b->position_x=20;
	b->position_y=10;
	b->word="Jon";
	b->next=NULL;

    qrNode* c=new qrNode;

	c->position_x=30;
	c->position_y=10;
	c->word="Kasper";
    c->next=NULL;

    qrNode* d=new qrNode;

	d->word="venu";
	d->position_x=40;
	d->position_y=20;

	qrLinkedList l;

	l.insertNode(a);
	l.insertNode(b);
	l.searchList(c->word);
	l.insertNode(d);

	ROS_INFO_STREAM("# of QR Codes: "<<l.getSize());
	ROS_INFO_STREAM("Node D: "<<"("<<d->position_x<<","<<d->position_y<<")");
	l.printWordList();
}
