
#include <cstdlib>
#include <iostream>

struct NavMeshSetHeader
{
	int magic;
	float val[2];
};

int main(int argc, char* argv[])
{
	float val = 16.0;
	char *a = new char[12];
	a[0] = (char)0x10;
	a[1] = (char)0x27;
	a[2] = (char)0;
	a[3] = (char)0;

	char *dv = (char*)&val;
	std::cout<<"double val:"<< dv << std::endl;

	a[4] = dv[0];
	a[5] = dv[1];
	a[6] = dv[2];
	a[7] = dv[3];
	a[8] = dv[0];
	a[9] = dv[1];
	a[10] = dv[2];
	a[11] = dv[3];


	
	NavMeshSetHeader* header = (NavMeshSetHeader*)a;


	std::cout<<"HelloWorld:"<< header->magic << "  "<< header->val[1] << "  ";
	return 0;
}
