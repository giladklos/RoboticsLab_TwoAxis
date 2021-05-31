#include <iostream>
#include<fstream>
#include<math.h>
#define l1 16
#define l2 6
#define l3 15
#define l4 15
#define l5 6
#define dt 0.4
using namespace std;
bool InBounds(float x,float y){
    float d= x-l1;
    if((pow(x,2)+pow(y,2))>(pow((l2+l3),2)))
    return false;
    else if((pow(x,2)+pow(y,2))<(pow((l2-l3),2)))
    return false;
    else if((pow(d,2)+pow(y,2))>(pow((l4+l5),2)))
    return false;
     else if((pow(d,2)+pow(y,2))<(pow((l4-l5),2)))
    return false;
    else return true;


}
float Opt(float ang1a, float ang1b,float theta){
    if(abs(ang1a-thata)<abs(ang1a-thata))
    return ang1;
    else
    return ang1b;
	
}
bool inverse_kinematics(float x, float y, float &ang1_1, float &ang2_1, float &ang1_2, float &ang2_2)
{
	//write inverse kinematics function here:
	//every error - return true
	//if invers kinematic is  ok - return false
	if(InBounds){
	/////////////////////////////////////////////////////
    //for theta 2
    float d1=Math.sqrt(Math.Pow(x,2)+Math.Pow(y,2));
    float d2=Math.sqrt(Math.Pow(x-l1,2)+Math.Pow(y,2));
    float beta2p=Math.acos((Math.pow(l2,2)+Math.pow(d1,2)-Math.pow(l3,2))/(2*l2*d1));
    float beta2m=-Math.acos((Math.pow(l2,2)+Math.pow(d1,2)-Math.pow(l3,2))/(2*l2*d1));
    float a2=Math.atan2(y,x);
    ang1_1=a2+beta2p;
    ang1_2=a2+beta2m;
     // for theta 5;
     float beta5p=Math.acos((Math.pow(l5,2)+Math.pow(d2,2)-Math.pow(l4,2))/(2*l5*d2));
    float beta5m=-Math.acos((Math.pow(l5,2)+Math.pow(d2,2)-Math.pow(l4,2))/(2*l5*d2));
    float a5=Math.atan2(y,(x-l1));
    ang2_1=a5+beta5p;
    ang2_2=a5+beta5m;
    return false;}
	//////////////////////////////////////////////////////
	return true;
}

void read_file(char file_name[], float x_vec[], float y_vec[], int *point_num)//read x y points from file
{
	std::ifstream infile;
	infile.open(file_name);
	if (!infile.is_open())
	{
		infile.close();
		cout << "Read file Error. Can not open the file. \n Press any key for exit" << endl;
		getchar();
		exit(1);
	}

	*point_num = 0;
	while (!infile.eof())//read up to end of the file
	{
		if (!(infile >> x_vec[*point_num] >> y_vec[*point_num]))
			break;
		(*point_num)++;
		if (*point_num >= 10000)
		{
			infile.close();
	
			cout << "Read file Error.Data points number > 9999.\n Press any key for exit" << endl;
			getchar();
			exit(1);
		}
	}
	infile.close();
	return;
}

void write_to_file(char file_name[], float ang1_vec[], float ang2_vec[], int point_num, float dt)
{
	//format: 
	//dt
	//ang1_vec[0], ang2_vec[0]
	//.
	//.
	//ang1_vec[point_num - 1],ang2_vec[point_num - 1]

	ofstream outfile;
	outfile.open(file_name);//open file
	
	if (outfile.is_open() == true)//if the file is open
	{
		outfile << dt << endl;
		for (int i = 0; i < point_num; i++)
		{
			outfile << ang1_vec[i] << "\t" << ang2_vec[i] << endl;
		}
		return;
	}
	
	cout << "Write file Error. Can not open the file.\n Press any key for exit" << endl;
	getchar();
	exit(1);
}




int main()
{
	//declarations
	char input_file_name[] =  "xy_input.txt";
	char output_file_name[] = "ang_output.txt";
	float dt = 0.1;//time delta between points

	float x_vec[10000] = { 0 }; // x vector
	float y_vec[10000] = { 0 }; // y vector
	float ang1_1vec[10000] = { 0 };//angle 1, first configuration
	float ang2_1vec[10000] = { 0 };//angle 2, first configuration
	float ang1vec[10000] = { 0 };//angle 1, final
	float ang2vec[10000] = { 0 };//angle 2, final
	float ang1_2vec[10000] = { 0 };//angle 1, second configuration
	float ang2_2vec[10000] = { 0 };//angle 2, second configuration
	int point_num = 0;

	//read work space data (x,y) from file, and check if error
	read_file(input_file_name, x_vec, y_vec, &point_num);

	//invers kinematics of all points
	for (int i = 0; i < point_num; i++)
	{
		//invers kinematics of one point get (x,y) return 2 possible angles for each axis 
		if (inverse_kinematics(x_vec[i], y_vec[i], ang1_1vec[i], ang2_1vec[i], ang1_2vec[i], ang2_2vec[i]))
		{
			cout << "inverse kinematics error - Press any key for exit\n ";
			getchar();
			exit(1);
			//cout << "angle 1,1: "<<ang1_1vec[i]<<"	angle 2,1: " << ang2_1vec[i]<<"	angle 1,2: " << ang1_2vec[i]<<"	angle 2,2: " << ang2_2vec[i] << endl;
		}
		if(i>0){
		ang1vec[i]=Opt(ang1_1vec[i],ang1_2vec[i],ang1vec[i-1]);
		ang2vec[i]=Opt(ang2_1vec[i],ang2_2vec[i],ang1vec[i-1]);
		}
		else {
			ang1vec[i]=ang1_1vec[i];
			ang2vec[i]=ang2_1vec[i];
		}
		if(x_vec[i]==0&&y_vec[i]==0){
			ang1vec[i]=ang1vec[i-1];
			ang2vec[i]=ang2vec[i-1];
		}
	}

	//write to file joint space data (file name,ang1, ang2, number of points, dt)
	write_to_file(output_file_name, ang1, ang2, point_num, dt);

	cout << "Inverse kinematics OK\n";
	system("pause");
	return 0;
}
