#include "Heuristics.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string>
#include <iostream>
#include <sstream>
#include <cstring>
#include <vector>
#include <fstream>
#include <cstdio>


using namespace std;

double** matrixTruck; //Represents the distance matrix for the truck (Manhattan distance)
double** matrixDrone; //Represents the distance matrix for the drone (Euclidean  distance)
deque<int> droneEligible; //Contains id's of drone eligible customers


vector<string> explode(const string& str, const char& ch) {
    string next;
    vector<string> result;

    // For each character in the string
    for (string::const_iterator it = str.begin(); it != str.end(); it++) {
        // If we've hit the terminal character
        if (*it == ch) {
            // If we have some characters accumulated
            if (!next.empty()) {
                // Add them to the result vector
                result.push_back(next);
                next.clear();
            }
        } else {
            // Accumulate the next character into the sequence
            next += *it;
        }
    }
    if (!next.empty())
         result.push_back(next);
    return result;
}

//Computes Euclidean distance given two nodes
double euclideanDistance(Node *n1, Node *n2){
    double val1, val2;

    val1 = n2->get_x()- n1->get_x();
    val2 = n2->get_y()- n1->get_y();

    val1 = pow(val1,2);
    val2 = pow(val2,2);

    return sqrt(val1 + val2);
}

//Computes Euclidean distance given two nodes
double manhattanDistance(Node *n1, Node *n2){
    double val1, val2;

    val1 = fabs(n1->get_x()- n2->get_x());
    val2 = fabs(n1->get_y()- n2->get_y());

    return (val1 + val2);
}

//Used for my generated instances
void initializeMatrix1(const char * filename){

    deque <Node*> all_nodes; //List of all nodes (N+2)
    int id_read = 0;
    double x_read = 0;
    double y_read = 0;
    int drone_read = 0;
    string line, p1, p2, p3, p4;
    int length = 0;

    all_nodes.clear();
    droneEligible.clear();

    ifstream read(filename); //open file

    if (read.is_open())
    {
        while (getline(read, line)) //go till end of file
        {
            std::istringstream csvStream(line);

            std::getline(csvStream, p1, ',');
            getline(csvStream, p2, ',');
            getline(csvStream, p3, ',');
            getline(csvStream, p4);

            id_read = atof(p1.c_str());
            x_read = atof(p2.c_str());
            y_read = atof(p3.c_str());
            drone_read = atof(p4.c_str());

            if(drone_read == 0)
                droneEligible.push_back(id_read);
            all_nodes.push_back(new Node(id_read, x_read, y_read, 1, drone_read));
        }
        droneEligible.pop_front();//we remove the origin and destination depot
        droneEligible.pop_back();


        read.close();
        length = all_nodes.size();

        //we create matrixTruck and matrixDrone
        matrixTruck = new double*[length];
        for(int i=0; i<length; i++)
            matrixTruck[i] = new double[length];

        matrixDrone = new double*[length];
        for(int i=0; i<length; i++)
            matrixDrone[i] = new double[length];

        //we fill matrixTruck and matrixDrone
        for(int i=0; i<length; i++){
            for(int j=0; j<length; j++){
                matrixTruck[i][j] = manhattanDistance(all_nodes[i], all_nodes[j]);
                matrixDrone[i][j] = euclideanDistance(all_nodes[i], all_nodes[j])/SPEED;
            }
        }

        std::for_each(all_nodes.begin(), all_nodes.end(), Delete());
        all_nodes.clear();

    }else{
        cout<<"error"<<endl;
    }

    //Ecriture dans fichier
    /*ofstream write1("Instances/Instances_att48/MatrixTruck.csv",ios::app);
    ofstream write2("Instances/Instances_att48/MatrixDrone.csv",ios::app);
    if (write1.is_open()){
        for(int i=0; i<length; i++){
            for(int j=0; j<length; j++){
                write1 << matrixTruck[i][j] <<";";
            }
            write1 << '\n';
        }
    }else
        cout<<"write1 pas ouvert"<<endl;

    if (write2.is_open()){
        for(int i=0; i<length; i++){
            for(int j=0; j<length; j++){
                write2 << matrixDrone[i][j] <<";";
            }
            write2 << '\n';
        }
    }else
        cout<<"write2 pas ouvert"<<endl;

    write1.close();
    write2.close();
*/
}

//Used for their instances

//find the size of tau file
int findSize(const char * tau){
    int lenght = 0;
    string line;
    ifstream read(tau); //open file

    if (read.is_open())
    {
        while (getline(read,line)) //go till end of file
        {
            std::vector<std::string> result = explode(line, ',');
            lenght = result.size();
            break;
        }
        read.close();
    }else
        cout<<"error"<<endl;
    return lenght;
}

//For Murray and Chu instances
void initializeMatrix2(const char * tau, const char * tauprime, const char * Cprime){

    int length = findSize(tau), length2;
    deque<int>::iterator it;
    //cout<< "length = "<<length<<endl;
    string line;
    ifstream read1(tau); //open file
    ifstream read2(tauprime); //open file
    ifstream read3(Cprime); //open file
    int k;

    //we create matrixTruck and matrixDrone

    matrixTruck = new double*[length];
    for(int i=0; i<length; i++)
        matrixTruck[i] = new double[length];

    matrixDrone = new double*[length];
    for(int i=0; i<length; i++)
        matrixDrone[i] = new double[length];

    //we fill matrixTruck and matrixDrone
    k=0;
    if (read1.is_open()) //reads tau file
    {
        while (getline(read1,line)) //go till end of file
        {
            std::vector<std::string> result = explode(line, ',');
            for(int j = 0; j<length; j++){
                matrixTruck[k][j] = atof(result[j].c_str());
            }
            k++;
        }
        read1.close();
    }else
        cout<<"error"<<endl;

    k = 0;
    if (read2.is_open()) //reads tauprime file
    {
        while (getline(read2,line)) //go till end of file
        {
            std::vector<std::string> result2 = explode(line, ',');
            for(int j = 0; j<length; j++){
                matrixDrone[k][j] = atof(result2[j].c_str())/SPEED;
            }
            k++;
        }
        read2.close();
    }else
        cout<<"error"<<endl;

    //we fill droneEligible list

    double speed = 25./60.;
    int id;
    if (read3.is_open()) //reads Cprime file
    {
        while (getline(read3,line)) //go till end of file
        {
            std::vector<std::string> result3 = explode(line, ',');
            length2 = result3.size();
            for(int j = 0; j<length2; j++){
                id = atof(result3[j].c_str());
                if(matrixDrone[0][id]*speed*2 <= 12.5)//(v=d/t => d=v*t max distance for a drone is d=(25./60.)*30)
                    droneEligible.push_back(id);
            }
        }
        read3.close();
    }else
        cout<<"error"<<endl;

    cout<<"la taille de drone eligible est "<<droneEligible.size()<<endl;
    cout<<"***** liste clients drone eligible *****"<<endl;
    for(it = droneEligible.begin(); it!=droneEligible.end(); ++it)
        cout<< *it<<",";
    cout<<endl;


}

//For instanceArticle
void initializeMatrix3(const char * tau, const char * tauprime, const char * Cprime){

    int length = findSize(tau), length2;
    //cout<< "length = "<<length<<endl;
    string line;
    ifstream read1(tau); //open file
    ifstream read2(tauprime); //open file
    ifstream read3(Cprime); //open file
    int k;

    //we create matrixTruck and matrixDrone

    matrixTruck = new double*[length];
    for(int i=0; i<length; i++)
        matrixTruck[i] = new double[length];

    matrixDrone = new double*[length];
    for(int i=0; i<length; i++)
        matrixDrone[i] = new double[length];

    //we fill matrixTruck and matrixDrone
    k=0;
    if (read1.is_open()) //reads tau file
    {
        while (getline(read1,line)) //go till end of file
        {
            std::vector<std::string> result = explode(line, ',');
            for(int j = 0; j<length; j++){
                matrixTruck[k][j] = atof(result[j].c_str());
            }
            k++;
        }
        read1.close();
    }else
        cout<<"error"<<endl;

    k = 0;
    if (read2.is_open()) //reads tauprime file
    {
        while (getline(read2,line)) //go till end of file
        {
            std::vector<std::string> result2 = explode(line, ',');
            for(int j = 0; j<length; j++){
                matrixDrone[k][j] = atof(result2[j].c_str());
            }
            k++;
        }
        read2.close();
    }else
        cout<<"error"<<endl;

    //we fill droneEligible list
    if (read3.is_open()) //reads Cprime file
    {
        while (getline(read3,line)) //go till end of file
        {
            std::vector<std::string> result3 = explode(line, ',');
            length2 = result3.size();
            for(int j = 0; j<length2; j++){
                droneEligible.push_back(atof(result3[j].c_str()));
            }
        }
        read3.close();
    }else
        cout<<"error"<<endl;

    cout<<"la taille de drone eligible est "<<droneEligible.size()<<endl;
}


void deleteMatrix(){
    delete[] matrixTruck;//we free the matrix
    delete[] matrixDrone;//we free the matrix
}

/********************************** HERE IS THE MAIN FUNCTION ***************************************/

int main(int argc, char* argv[])
{
    string path, file_n, str;
    const char * filename;

    /********** MY INSTANCES  **********/

    if(argc > 1)
        file_n = argv[1];
    else
        file_n = "att48_0_80.csv";

    std::vector<std::string> result = explode(file_n, '_');

    path = "Instances/Instances_"+result[0]+"/";
    str = path+file_n;
    filename = str.c_str();

    //We initialize the distances matrix
    initializeMatrix1(filename);

    /******** THEIR INSTANCES **********/
/*    path = "Instances/PDSTSP_10_customer_problems/";
    //path = "Instances/PDSTSP_20_customer_problems/";

    string tau;
    string tauprime;
    string Cprime;
    string nodes;

    if(argc > 1){
         tau = argv[1];
         tauprime = argv[2];
         Cprime = argv[3];
         nodes = argv[4];
    }else{

         tau = "instanceArticle/tau.csv";
         tauprime = "instanceArticle/tauprime.csv";
         Cprime = "instanceArticle/Cprime.csv";
         nodes = "instanceArticle/nodes.csv";


         //For 10 customers
         tau = "20140813T111703/tau.csv";
         tauprime = "20140813T111703/tauprime.csv";
         Cprime = "20140813T111703/Cprime.csv";
         nodes = "20140813T111703/nodes.csv";


        //For 20 customers
         tau = "20140813T125334/tau.csv";
         tauprime = "20140813T125334/tauprime.csv";
         Cprime = "20140813T125334/Cprime.csv";
         nodes = "20140813T125334/nodes.csv";

    }

    str = path+nodes;
    filename = str.c_str();
    std::vector<std::string> result = explode(tau, '/');
    file_n = result[0];

    //Murray's instances initialization
    initializeMatrix2((path+tau).c_str(),(path+tauprime).c_str(),(path+Cprime).c_str());

    //Article instance initialization
    //initializeMatrix3((path+tau).c_str(),(path+tauprime).c_str(),(path+Cprime).c_str());
*/

    freopen("trace_4_5.txt","w",stdout);

    /******** WE RUN OUR HEURISTIC **********/
    srand(time(NULL));
    Heuristics *pdstsp = new Heuristics(filename);
    cout<<"====Debut du programme===="<<endl;
    //pdstsp->run_heuristic(path,file_n);
    //pdstsp->run_heuristic2(path,file_n);
    pdstsp->run_ILS(path,file_n);
    //pdstsp->statAnalysis(path,file_n);
    //pdstsp->limitLabelAnalysis(path,file_n);

    cout<<endl;
    cout<<"====Fin du programme===="<<endl;
    deleteMatrix();

    //delete pdstsp;

    return 0;

}
