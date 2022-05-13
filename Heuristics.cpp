#include "Heuristics.h"
#include <iostream>
#include <algorithm>
#include <time.h>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include <limits>
#include <string>
#include <ctime>
#include <ratio>
#include <chrono>

using namespace std;

//Constructor, takes filename to read from as input
Heuristics::Heuristics(const char * file)
{
    filename = file;

    /*We distinguish two reading method because for MURRAY and CHU file we have
    to determine drone eligible customers (drone endurance : 30 mins)*/

    //read_file1(file); //reads MURRAY and CHU instances node files
    read_file2(file); //reads our instances files

}

//Destructor clears the deques
Heuristics::~Heuristics()
{
    deque<deque <int> >::iterator it;

    std::for_each(all_customers.begin(), all_customers.end(), Delete());
    all_customers.clear();

    truck_tour.clear();

    drone_list.clear();

    best_truck_tour.clear();

    best_drone_list.clear();

    for(it = drone_assignment.begin(); it!=drone_assignment.end(); ++it)
        (*it).clear();
    drone_assignment.clear();

    for(it = best_drone_assignment.begin(); it!=best_drone_assignment.end(); ++it)
        (*it).clear();
    best_drone_assignment.clear();

    bsup1_truck_tour.clear();

    bsup2_truck_tour.clear();

    bsup1_drone_list.clear();

    bsup1_drone_list.clear();

    delete recor_label;
    delete bsup1_label;
    delete bsup2_label;

    std::for_each(tour.begin(), tour.end(), Delete());
    tour.clear();
}

//Read customers data from file into heuristics's members (their instances)
void Heuristics::read_file1(const char * filename)
{
    int id_read = 0, taille;
    double x_read = 0;
    double y_read = 0;
    int drone_read = 0;
    double speed;

    string line, p1, p2, p3, p4;

    all_customers.clear();

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

            all_customers.push_back(new Node(id_read, x_read, y_read, 1, drone_read));
        }

        taille = all_customers.size();
        speed = 25./60.;

        int nb = 0;
        for(int i=1; i<taille-1;i++){
             if((all_customers[i]->get_drone() == 0)&&(matrixDrone[all_customers[i]->get_id()][taille-1]*speed*2 > 12.5)){
               all_customers[i]->set_drone(1);
                nb++;
            }else{
                if(all_customers[i]->get_drone() == 1)
                    nb++;
            }

        }

        //cout<<"le nombre de client drone eligible à la fin est "<<nb<<endl;

        all_customers.pop_back(); //we remove the destination depot node
        all_customers[0]->set_drone(1);//the depot is imposed to the truck
        read.close();
    }else{
        cout<<"error"<<endl;
    }

    //cout<<"je suis entree ici"<<endl;
}


//Read customers data from file into heuristics's members (our instances)
void Heuristics::read_file2(const char * filename)
{
    int id_read = 0;
    double x_read = 0;
    double y_read = 0;
    int drone_read = 0;

    string line, p1, p2, p3, p4;

    all_customers.clear();

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

            all_customers.push_back(new Node(id_read, x_read, y_read, 1, drone_read));
        }

        all_customers.pop_back(); //we remove the destination depot node
        all_customers[0]->set_drone(1);//the depot is imposed to the truck
        read.close();
    }else{
        cout<<"error"<<endl;
    }
}

//Returns true if label l is dominated by label p
bool Heuristics::dominance_test(Label *l, Label *p){
    if((l->c1>=p->c1 && l->c2>p->c2)||(l->c1>p->c1 && l->c2>=p->c2))
        return true;
    else
        return false;

}

//Returns true if label l is dominated by label p (case with K vehicles)
bool Heuristics::dominance_test2(Label2 *l, Label2 *p, int id){
    if(maxi(p->c_gloVeh,p->c1+matrixTruck[0][id],p->c2) <= maxi(l->c_gloVeh,l->c1+matrixTruck[0][id],l->c2) &&
       p->c1 <= l->c1 && p->c2 <= l->c2 && p->numVeh <= l->numVeh)
        return true;
    else
        return false;

}


//Returns upper bound proposed by ALAIN
double Heuristics::bSup1(deque <Node*> tour){

    double V=0, D=0;
    int i=0, j=i+1, taille = tour.size();

    bsup1_label = new Label();

    bsup1_truck_tour.clear();
    bsup1_drone_list.clear();
    bsup1_truck_tour.push_back(tour[0]->get_id());
    while(j<taille){
        if ((tour[j]->get_drone() == 0) && (max(V,(D+(matrixDrone[0][tour[j]->get_id()])*2)/NB_DRONE) < max(V,D))){
            bsup1_drone_list.push_back(tour[j]->get_id());
            D = D+(matrixDrone[0][tour[j]->get_id()])*2/NB_DRONE;
        }else{
            bsup1_truck_tour.push_back(tour[j]->get_id());
            V = V + matrixTruck[tour[i]->get_id()][tour[j]->get_id()];
            i = j;
        }
        j = j+1;
    }

    bsup1_label->c1 = V;
    bsup1_label->c2 = D;

    return max(V,D);
}

//Solve the problem 1 vehicle 1 drone considering acceleration coef and returns the vehicle tour
deque<int> Heuristics::solveSimplePb(deque <int> &tourN, double coef){

    bool stop, added=true;
    int length = tourN.size(), j;
    double S;
    Label *l, *best_label;
    deque<Label*>::iterator it, it2;  //An iterator on labels list
    deque<Node*>::iterator it3;  //An iterator on Node list
    deque<Node*> tour;
    deque<int> vehTour;//will contain the vehicle after the split

    //cout<<"*********** solveSimplePb coef = "<<coef<<" ************"<<endl;
    bsup3_label = new Label();

    //tour initialization into node list
    tour.clear();
    for(int i=0; i<length-1; i++){
        tour.push_back(find_node_by_id(tourN[i]));
        //cout<<tour[i]->get_id()<<";";
    }
    //cout<<endl;
    tour.push_back(new Node(*tour[0]));

    //cout<<"la taille de tour dans solveSimplePb est "<<tour.size()<<endl;
    //we ensure that the label list of each node in tour is empty
    for(int i=0; i<length; i++){
        std::for_each(tour[i]->labels.begin(), tour[i]->labels.end(), Delete());
        tour[i]->labels.clear();
    }

    l = new Label();
    l->current_node_id = tour[0]->get_id();
    l->c1 = 0;
    l->c2 = 0;
    l->father = NULL;

    tour[0]->labels.push_back(l); //we but label (0,0) in the label list of tour[0]
    //cout<<"length = "<<length<<endl;
    for(int i=1; i<length; i++){

        //if(is_drone_eligible(tour[i]->get_id()))
        //        cout<<tour[i]->get_id() <<" est DE"<<endl;
        //cout<<"i = "<<i<<endl;
        stop = false;
        S = 0; //will contain the cost related to the drone
        j = i-1; //j is the predecessor of i in tour

        while((stop == false)&&(j>=0)){
            for(it = tour[j]->labels.begin(); it!=tour[j]->labels.end(); ++it){
                l = new Label();
                l->c1 = (*it)->c1 + (matrixTruck[tour[j]->get_id()][tour[i]->get_id()]/coef);
                l->c2 = (*it)->c2 + S/NB_DRONE;
                l->current_node_id = tour[i]->get_id();
                l->father = (*it);
                //cout<<"label cree ("<<l->c1<<","<<l->c2<<")"<<endl;

                added = true; //will verify if created label is added

                //We try to add l to tour[i]->labels list
                it2 = tour[i]->labels.begin();
                while(it2 !=tour[i]->labels.end()){
                    if(max(l->c1,l->c2) >= max((*it2)->c1,(*it2)->c2)){ //the label pointed by it is better than l (strong dominance)
                         added = false;
                         delete l;
                         break;
                    }else{ //true if the label pointed by it is dominated by l

                        //cout<<"on supprime ("<<(*it2)->c1<<","<<(*it2)->c2<<") "<<endl;
                        delete (*it2);
                        it2 = tour[i]->labels.erase(it2);
                    }
                }
                if(added){ //we add l to tour[i]->labels list
                    tour[i]->labels.push_back(l);
                    //cout<<"i = "<<i<<endl;
                    //cout<<"label ajoute ("<<l->c1<<","<<l->c2<<")"<<endl;

                }
            }

            if ((tour[j]->get_id() == 0) || !is_drone_eligible(tour[j]->get_id())) //if j is not drone eligible or j is the origin depot
                stop = true;
            else{
                S = S + (matrixDrone[0][tour[j]->get_id()])*2;
                j = j-1; //we go to the next predecessor
            }
        }
        //if(tour[i]->labels.size()>=1)
        //    cout<<"labels du noeud "<< i <<" : ("<<tour[i]->labels[0]->c1<<","<<tour[i]->labels[0]->c2<<")"<<endl;

    }

    best_label = tour[length-1]->labels[0]; //the label in depot node labels list at the end

    //cout<<"best_label->c1 = "<<best_label->c1<<endl;
    //cout<<"best_label->c2 = "<<best_label->c2<<endl;

    //we keep the solution related to bsup

    vehTour.clear();
    vehTour.push_front(tour[length-1]->get_id());//we put the destination depot id in the truck_solution_tour
    Label * father;
    father = best_label->father;
    while(father != NULL){
        if(father->current_node_id != 0)
            vehTour.push_front(father->current_node_id);//we add
        else
            vehTour.push_front(tour[0]->get_id());
        father = father->father;
    }

    bsup3_drone_list.clear();
    for(it3 = all_customers.begin(); it3!=all_customers.end(); ++it3){
        if(!is_in_truck_tour((*it3)->get_id(), vehTour))
            bsup3_drone_list.push_back((*it3)->get_id());
    }

    bsup3_label->c2 = best_label->c2;

    //cout<<"la taille de vehTour est "<<vehTour.size()<<endl;
    return vehTour;

/*
    deque<int> vehTour;
    double V=0, D=0;
    int i=0, j=i+1, taille = tour.size();

    bsup_label = new Label();

    vehTour.clear();
    bsup_drone_list.clear();
    vehTour.push_back(tour[0]);
    while(j<taille){
        if ((find_node_by_id(tour[j])->get_drone() == 0) && (max(V,(D+(matrixDrone[0][tour[j]])*2)/NB_DRONE) < max(V,D))){
            bsup_drone_list.push_back(tour[j]);
            D = D+(matrixDrone[0][tour[j]])*2/NB_DRONE;
        }else{
            vehTour.push_back(tour[j]);
            V = V + (matrixTruck[tour[i]][tour[j]]/coef);
            i = j;
        }
        j = j+1;
    }

    //bsup1_label->c1 = V;
    bsup_label->c2 = D;

    return vehTour;
*/
}


//Returns upper bound proposed by DOMINIQUE
double Heuristics::bSup2(deque <Node*> &tour){

    bool stop, added=true;
    int length = tour.size(), j;
    double S;
    Label *l, *best_label;
    deque<Label*>::iterator it, it2;  //An iterator on labels list
    deque<Node*>::iterator it3;  //An iterator on Node list

    bsup2_label = new Label();

    //we ensure that the label list of each node in tour is empty
    for(int i=0; i<length; i++){
        std::for_each(tour[i]->labels.begin(), tour[i]->labels.end(), Delete());
        tour[i]->labels.clear();
    }

    l = new Label();
    l->current_node_id = tour[0]->get_id();
    l->c1 = 0;
    l->c2 = 0;
    l->father = NULL;

    tour[0]->labels.push_back(l); //we but label (0,0) in the label list of tour[0]

    for(int i=1; i<length; i++){
        stop = false;
        S = 0; //will contain the cost related to the drone
        j = i-1; //j is the predecessor of i in tour

        while((stop == false)&&(j>=0)){
            for(it = tour[j]->labels.begin(); it!=tour[j]->labels.end(); ++it){
                l = new Label();
                l->c1 = (*it)->c1 + matrixTruck[tour[j]->get_id()][tour[i]->get_id()];
                l->c2 = (*it)->c2 + S/NB_DRONE;
                l->current_node_id = tour[i]->get_id();
                l->father = (*it);

                added = true; //will verify if created label is added

                //We try to add l to tour[i]->labels list
                it2 = tour[i]->labels.begin();
                while(it2 !=tour[i]->labels.end()){
                    if(max(l->c1,l->c2) >= max((*it2)->c1,(*it2)->c2)){ //the label pointed by it is better than l (strong dominance)
                         added = false;
                         delete l;
                         break;
                    }else{ //true if the label pointed by it is dominated by l
                        delete (*it2);
                        it2 = tour[i]->labels.erase(it2);
                    }
                }
                if(added){ //we add l to tour[i]->labels list
                    tour[i]->labels.push_back(l);
                }
            }

            if ((tour[j]->get_id() == 0) || !is_drone_eligible(tour[j]->get_id())) //if j is not drone eligible or j is the origin depot
                stop = true;
            else{
                S = S + (matrixDrone[0][tour[j]->get_id()])*2;
                j = j-1; //we go to the next predecessor
            }
            //cout<<"la liste de labels de tour de i a pour taille "<<tour[i]->labels.size()<<endl;
        }

    }

    best_label = tour[length-1]->labels[0]; //the label in depot node labels list at the end

    //we keep the solution related to bsup2

    bsup2_truck_tour.clear();
    bsup2_truck_tour.push_front(tour[length-1]->get_id());//we put the destination depot id in the truck_solution_tour
    Label * father;
    father = best_label->father;
    while(father != NULL){
        if(father->current_node_id != 0)
            bsup2_truck_tour.push_front(father->current_node_id);//we add
        else
            bsup2_truck_tour.push_front(tour[0]->get_id());
        father = father->father;
    }

    bsup2_drone_list.clear();
    for(it3 = all_customers.begin(); it3!=all_customers.end(); ++it3){
        if(!is_in_truck_tour((*it3)->get_id(), bsup2_truck_tour))
            bsup2_drone_list.push_back((*it3)->get_id());
    }

    bsup2_label->c1 = best_label->c1;
    bsup2_label->c2 = best_label->c2;

    return max(best_label->c1,best_label->c2);
}

//bSup1 for the case with k vehicles
double Heuristics::bSup3(deque <int> &tourN){

    deque<int>::iterator it;
    double val;
    //cout<<"*********** bSup3 **************"<<endl;
    double coef = decompose(tourN, bsup3_trucks_tours, 0);
    //cout<<"le coef d'acceleration est "<<coef<<endl;
    //cout<<"on sort du premier decompose"<<endl;

    deque<int> tour = solveSimplePb(tourN,coef);
    //cout<<"le tour vehicule obtenu apres solveSimplePb est "<<endl;
    /*for(it=tour.begin();it!=tour.end();++it)
        cout<<*it<<endl;
    cout<<endl;*/
    //cout<<"on sort de solveSimplePb"<<endl;

    tour.pop_back();
    tour.push_back(tourN.size()-1);
    decompose(tour,bsup3_trucks_tours,1);

    //we search for the cost of the longest vehicle tour
    int taille = bsup3_trucks_tours.size(), i=1;
    bsup3_label->c1 = get_truck_cost(bsup3_trucks_tours[0]);
    while(i<taille){
        val = get_truck_cost(bsup3_trucks_tours[i]);
        if(val> bsup3_label->c1)
            bsup3_label->c1 = val;
        i++;
    }

    /*//VERIFICATION
    for(it=bsup3_drone_list.begin();it!=bsup3_drone_list.end(); ++it)
        cout<<*it<<";";
    cout<<endl;*/

    return max(bsup3_label->c1,bsup3_label->c2);
}

//bSup2 for the case with k vehicles (a vehicle that goes K times faster)
double Heuristics::bSup4(deque <int> &tourN){
    bool stop, added=true;
    int length = tourN.size(), j, taille;
    double S, val;
    Label *l, *best_label;
    deque<Label*>::iterator it, it2;  //An iterator on labels list
    deque<Node*>::iterator it3;  //An iterator on Node list
    deque<Node*> tour;
    deque<int> vehTour;//will contain the vehicle after the split

    //cout<<"*********** bSup4 *************"<<endl;

    bsup4_label = new Label();

    //tour initialization into node list
    tour.clear();
    for(int i=0; i<length-1; i++){
        tour.push_back(find_node_by_id(tourN[i]));
    }
    tour.push_back(new Node(*tour[0]));

    //we ensure that the label list of each node in tour is empty
    for(int i=0; i<length; i++){
        std::for_each(tour[i]->labels.begin(), tour[i]->labels.end(), Delete());
        tour[i]->labels.clear();
    }

    l = new Label();
    l->current_node_id = tour[0]->get_id();
    l->c1 = 0;
    l->c2 = 0;
    l->father = NULL;

    tour[0]->labels.push_back(l); //we but label (0,0) in the label list of tour[0]
    for(int i=1; i<length; i++){
        stop = false;
        S = 0; //will contain the cost related to the drone
        j = i-1; //j is the predecessor of i in tour

        while((stop == false)&&(j>=0)){
            for(it = tour[j]->labels.begin(); it!=tour[j]->labels.end(); ++it){
                l = new Label();
                l->c1 = (*it)->c1 + (matrixTruck[tour[j]->get_id()][tour[i]->get_id()]/NB_VEH);
                l->c2 = (*it)->c2 + S/NB_DRONE;
                l->current_node_id = tour[i]->get_id();
                l->father = (*it);
                //cout<<"label cree ("<<l->c1<<","<<l->c2<<")"<<endl;

                added = true; //will verify if created label is added

                //We try to add l to tour[i]->labels list
                it2 = tour[i]->labels.begin();
                while(it2 !=tour[i]->labels.end()){
                    if(max(l->c1,l->c2) >= max((*it2)->c1,(*it2)->c2)){ //the label pointed by it is better than l (strong dominance)
                         added = false;
                         delete l;
                         break;
                    }else{ //true if the label pointed by it is dominated by l

                        //cout<<"on supprime ("<<(*it2)->c1<<","<<(*it2)->c2<<") "<<endl;
                        delete (*it2);
                        it2 = tour[i]->labels.erase(it2);
                    }
                }
                if(added){ //we add l to tour[i]->labels list
                    tour[i]->labels.push_back(l);
                    //cout<<"i = "<<i<<endl;
                    //cout<<"label ajoute ("<<l->c1<<","<<l->c2<<")"<<endl;

                }
            }

            if ((tour[j]->get_id() == 0) || !is_drone_eligible(tour[j]->get_id())) //if j is not drone eligible or j is the origin depot
                stop = true;
            else{
                S = S + (matrixDrone[0][tour[j]->get_id()])*2;
                j = j-1; //we go to the next predecessor
            }
            //cout<<"la liste de labels de tour de "<< i <<" a pour taille "<<tour[i]->labels.size()<<endl;
        }
        //if(tour[i]->labels.size()>=1)
        //    cout<<"labels du noeud : "<< i <<"("<<tour[i]->labels[0]->c1<<","<<tour[i]->labels[0]->c2<<")"<<endl;

    }

    best_label = tour[length-1]->labels[0]; //the label in depot node labels list at the end

    //cout<<"best_label->c1 = "<<best_label->c1<<endl;
    //cout<<"best_label->c2 = "<<best_label->c2<<endl;

    //we keep the solution related to bsup

    vehTour.clear();
    vehTour.push_front(tour[length-1]->get_id());//we put the destination depot id in the truck_solution_tour
    Label * father;
    father = best_label->father;
    while(father != NULL){
        if(father->current_node_id != 0)
            vehTour.push_front(father->current_node_id);//we add
        else
            vehTour.push_front(tour[0]->get_id());
        father = father->father;
    }

    bsup4_drone_list.clear();
    for(it3 = all_customers.begin(); it3!=all_customers.end(); ++it3){
        if(!is_in_truck_tour((*it3)->get_id(), vehTour))
            bsup4_drone_list.push_back((*it3)->get_id());
    }

    bsup4_label->c2 = best_label->c2;

    decompose(vehTour, bsup4_trucks_tours,1);
    //we search for the cost of the longest vehicle tour
    taille = bsup4_trucks_tours.size();
    int i=1;
    bsup4_label->c1 = get_truck_cost(bsup4_trucks_tours[0]);
    while(i<taille){
        val = get_truck_cost(bsup4_trucks_tours[i]);
        if(val> bsup4_label->c1)
            bsup4_label->c1 = val;
        i++;
    }

    //cout<<"la taille de vehTour est "<<vehTour.size()<<endl;
    return max(bsup4_label->c1,bsup4_label->c2);

}

//Computes lower bound for each node given a sequence tour
double* Heuristics::bInf(deque <Node*> tour, double bSup, int option){
    int taille = tour.size(), j;
    double* bInf_table_somme;
    double* bInf_table_vehicle;
    double val1,val2, S;
    bool stop;

    bInf_table_somme = new double[taille];
    bInf_table_vehicle = new double[taille];
    bInf_table_somme[taille-1] = 0;
    bInf_table_vehicle[taille-1] = 0;

    for(int i=taille-2; i>=0; i--){
        stop = false;
        S = 0;
        j = i+1;
        val1 = bInf_table_somme[j]+matrixTruck[tour[i]->get_id()][tour[j]->get_id()]+S/NB_DRONE;
        val2 = bInf_table_vehicle[j]+matrixTruck[tour[i]->get_id()][tour[j]->get_id()];

        while(!stop && (j<=taille-1)){
            val1 = min(val1, bInf_table_somme[j]+matrixTruck[tour[i]->get_id()][tour[j]->get_id()]+S/NB_DRONE);
            val2 = min(val2, bInf_table_vehicle[j]+matrixTruck[tour[i]->get_id()][tour[j]->get_id()]);

           if((j==taille -1) || (tour[j]->get_drone() != 0)) //j is drone eligible or is the destination depot
                stop = true;
           else{
                S = S + (matrixDrone[0][tour[j]->get_id()])*2;
                j = j+1;
           }
        }
        bInf_table_somme[i] = val1;
        bInf_table_vehicle[i] = val2;
    }

    /*for(int i=1; i<taille; i++)
        cout<< bInf_table_vehicle[i]<<";";
    cout<<endl;*/

    if (option  == 1)
        return bInf_table_vehicle;
    else
        return bInf_table_somme;
}

double** Heuristics::preprocess(deque<int> &tour){

    int n, x, indice, j, y, best_y;
    double val = 0, best_val;
    int destDepot = all_customers.size();
    double **V;
    deque<int>::iterator it;

    //cout<<"*********** Preprocess ************"<<endl;
    //firstly we remove DE customers in tour
    it = tour.begin()+1;
    while(it!=tour.end()-1){
        if(is_drone_eligible(*it))
            it = tour.erase(it);
        else
            it++;
    }
    n = tour.size();

    //cout<<"la taille du tour sans clients DE est "<<n<<endl;

    //we inverse tour such as to be able to apply the algorithm decompose exactly
    reverse(tour.begin()+1, tour.end()-1);

    V = new double*[n];
    for(int i=0; i<n; i++)
       V[i] = new double [NB_VEH];

    //initialization
    for(it = tour.begin(); it!=tour.end(); ++it){
        //cout <<"it = "<<*it<<endl;
        //cout<<"lengthPath(tour,destDepot,"<<*it<<") = "<<lengthPath(tour,0,*it)<<endl;
        V[std::distance(tour.begin(),it)][0] = lengthPath(tour,0,*it) + matrixTruck[*it][0];
        //cout<<"V*["<<*it<<"][0] = "<<V[std::distance(tour.begin(),it)][0]<<endl;
    }

    if(NB_VEH >=2){
        for(int i=1; i<NB_VEH; i++)
            V[0][i] = 0;

        for(int i=1; i<NB_VEH; i++){
            indice = 1;
            x = tour[indice]; //succ(0)
            //cout<<"x = "<<x<<endl;
            V[indice][i] = matrixTruck[0][x] + matrixTruck[x][destDepot];
            //cout<<"V*["<<indice<<"]["<<i<<"] = "<<V[indice][i]<<endl;
            indice++;
            while(x != destDepot && indice < n){
                best_val = std::numeric_limits<double>::max();
                x = tour[indice]; // x<- succ(x)
                //we go through x predecessors
                j = indice -1;
                //cout<<"x = "<<x<<endl;
                while(j>=0){
                    y = tour[j];
                    //cout<<"y = "<<y<<endl;
                    val = max(V[j][i-1], matrixTruck[0][tour[j+1]] + lengthPath(tour, tour[j+1],x) + matrixTruck[x][destDepot]);
                    //cout<<"val = "<<val<<endl;
                    if (val < best_val){
                        best_val = val;
                        best_y = j;
                    }
                    j--;
                }
                //cout<<"best_val = "<<best_val<<endl;
                //cout<<"best_y = "<<best_y<<endl;
                V[indice][i] = best_val;
                //cout<<"V*["<<indice<<"]["<<i<<"] = "<<V[indice][i]<<endl;
                indice++;
            }
        }

    }

    //arrange V
    int i = 0, k = n-1;
    double aux;
    while(i<k){
        j=0;
        while(j<NB_VEH){
            aux = V[i][j];
            V[i][j] = V[k][j];
            V[k][j] = aux;

            j++;
        }
        i++;
        k--;
    }

    //verification
    /*cout<<"Voici le tableau V*"<<endl;
    cout<<endl;

    for(int i=0;i<n; i++){
        for(int j=0;j<NB_VEH;j++)
            cout<<"V*["<<i<<"]["<<j<<"] = "<<V[i][j]<<";";
        cout<<endl;
    }
    */

    return V;
}

//bInf in the case with K vehicles considering a label l
double Heuristics::bInf(deque <Node*> tour,int indice, double** V, Label2* l){
    double val, val2, binf;
    int i,j,j_prim, destDepotId = all_customers.size(), taille = tour.size(), index;
    deque <int> tourN;
    deque<int>::iterator it;
    deque<Node*>::iterator it2;
    bool stop;

    tourN.clear();
    tourN.push_back(tour[0]->get_id());

    //cout<<"*********** bInf ************"<<endl;

    //cout<<"l = ("<<l->c_gloVeh<<","<<l->c1<<","<<l->c2<<","<<l->numVeh<<")"<<endl;

    for(it2=tour.begin()+1; it2!=tour.end()-1;++it2)
        if(!is_drone_eligible((*it2)->get_id()))
            tourN.push_back((*it2)->get_id());

    tourN.push_back(destDepotId);

    i = l->current_node_id;
    j = i;
    binf = std::numeric_limits<double>::max();

    while(j<destDepotId){
        val2 = 0;
        if (!is_drone_eligible(j)){
            j_prim = succ(tourN,j);
            //cout<<"j est impose o vehicule et j' = "<<j_prim<<endl;
        }else{
            index = indice;
            index++;
            stop = false;
            while(index < taille && !stop){
                if(!is_drone_eligible(tour[index]->get_id()))
                    stop = true;
                else
                    index++;
            }
            j_prim = tour[index]->get_id();
        }

        //cout<<"i = "<<i<<", j = "<<j<<" et j' = "<<j_prim<<endl;

        if(j==i){
            val = l->c1 + matrixTruck[j][destDepotId];
            //cout<<"val = "<<val<<endl;
        }else{
            if(!is_drone_eligible(i)){
                val = l->c1 + lengthPath(tourN,i,j) + matrixTruck[j][destDepotId];
                //cout<<"val = "<<val<<endl;
            }else{
                index = indice;
                index++;
                stop = false;
                while(index < taille && !stop){
                    if(!is_drone_eligible(tour[index]->get_id()))
                        stop = true;
                    else
                        index++;
                }

                val = l->c1 + matrixTruck[i][tour[index]->get_id()] + lengthPath(tourN,tour[index]->get_id(),j);
                //cout<<"val ici = "<<val<<endl;
            }
        }
        val2 = max(V[getIndex(tourN,j_prim)][NB_VEH-l->numVeh-1] , val);

        //cout<<"Max = "<<val2<<endl;
        if(val2 < binf)
            binf = val2;

        j = j_prim;
        j_prim = succ(tourN,j_prim);
    }
    if(i == destDepotId){
        binf = max(l->c_gloVeh,l->c1);
    }

    //cout<<"binf = "<<binf<<endl;
    return binf;
}


//Decomposes tour in NB_VEH tours and returns the acceleration coefficient
double Heuristics::decompose(deque<int> &tour, deque< deque<int> > &sol_trucks_tours,int option){

    int n = tour.size(), x, indice, j, y, best_y;
    double val = 0, best_val;
    int destDepot = all_customers.size();
    deque<int>::iterator it;
    double **V;
    int **Arg;

    //cout<<"*********** decompose option = "<<option<<" ************"<<endl;
    V = new double*[n];
    Arg = new int*[n];
    for(int i=0; i<n; i++)
       V[i] = new double [NB_VEH];
    for(int i=0; i<n; i++)
       Arg[i] = new int [NB_VEH];


    //initialization
    for(it = tour.begin(); it!=tour.end(); ++it){
        //cout <<"it = "<<*it<<endl;
        V[std::distance(tour.begin(),it)][0] = lengthPath(tour,0,*it) + matrixTruck[*it][destDepot];
        //cout<<"V["<<std::distance(tour.begin(),it)<<"][0] = "<<V[std::distance(tour.begin(),it)][0]<<endl;
    }

    //cout<<"taille du tour = "<<n<<endl;
    if(NB_VEH >=2){
        for(int i=1; i<NB_VEH; i++)
            V[0][i] = 0;

        for(int i=1; i<NB_VEH; i++){
            indice = 1;
            x = tour[indice]; //succ(0)
            //cout<<"x = "<<x<<endl;
            indice++;
            V[indice][i] = matrixTruck[0][x] + matrixTruck[x][destDepot];
            //cout<<"V[x][i] = "<<V[indice][i]<<endl;
            while(x != destDepot && indice < n){
                best_val = std::numeric_limits<double>::max();
                x = tour[indice]; // x<- succ(x)
                //we go through x predecessors
                j = indice -1;
                //cout<<"x = "<<x<<endl;
                while(j>=0){
                    y = tour[j];
                    //cout<<"y = "<<y<<endl;
                    val = max(V[j][i-1], matrixTruck[0][tour[j+1]] + lengthPath(tour, tour[j+1],x) + matrixTruck[x][destDepot]);
                    //cout<<"val = "<<val<<endl;
                    if (val < best_val){
                        best_val = val;
                        best_y = j;
                    }
                    j--;
                }
                //cout<<"best_val = "<<best_val<<endl;
                //cout<<"best_y = "<<best_y<<endl;
                V[indice][i] = best_val;
                Arg[indice][i] = best_y;

                indice++;
            }
        }

    }

    if(option == 1){//we one to retrieve vehicles tours and update sol_trucks_tours
        //cout<<"option = 1"<<endl;
        sol_trucks_tours.clear();
        deque<int> vehTour;
        int nb=NB_VEH, id, prev_id;
        bool trouve;
        while(nb!=0){
            if(nb == NB_VEH){//we are constructing the last vehicle tour
                id = Arg[n-1][nb-1];
                it = tour.begin();
                trouve = false;
                vehTour.clear();
                vehTour.push_back(0);
                while(it!=tour.end() && !trouve){
                    if(*it == tour[id])
                        trouve = true;
                    else
                        it++;
                }
                if (trouve){
                    it++;
                    while(it!=tour.end()-1){
                        //cout<<*it<<endl;
                        vehTour.push_back(*it);
                        it++;
                    }
                }

                vehTour.push_back(0);
                sol_trucks_tours.push_front(vehTour);

               /* //verification
                cout<<"Voici le tour du vehicule "<<nb<<endl;
                for(it=vehTour.begin(); it!= vehTour.end();++it)
                    cout<<*it<<";";
                cout<<endl;*/
            }

            if(nb>1 && nb != NB_VEH){
                prev_id = id;
                it=tour.begin();
                vehTour.clear();
                id = Arg[prev_id][nb-1];
                trouve = false;
                vehTour.push_back(0);

                while(it!=tour.end() && !trouve){
                    if(*it == tour[id])
                        trouve = true;
                    else
                        it++;
                }
                if (trouve){
                    it++;
                    while(*it!=tour[prev_id] && it!= tour.end()){
                        vehTour.push_back(*it);
                        it++;
                    }
                    vehTour.push_back(tour[prev_id]);
                }
                vehTour.push_back(0);
                sol_trucks_tours.push_front(vehTour);
                /*//verification
                cout<<"Voici le tour du vehicule "<<nb<<endl;
                for(it=vehTour.begin(); it!= vehTour.end();++it)
                    cout<<*it<<";";
                cout<<endl;*/

            }
            //cout<<"nb = "<<nb<<endl;
            //cout<<"on arrive ici"<<endl;
            if(nb == 1){//we build the tour of the first vehicle
                prev_id = id;
                it=tour.begin();
                vehTour.clear();
                while(*it != tour[prev_id] && it!= tour.end()){
                    vehTour.push_back(*it);
                    it++;
                }
                vehTour.push_back(tour[prev_id]);
                vehTour.push_back(0);
                sol_trucks_tours.push_front(vehTour);

                /*//verification
                cout<<"Voici le tour du vehicule "<<nb<<endl;
                for(it=vehTour.begin(); it!= vehTour.end();++it)
                    cout<<*it<<";";
                cout<<endl;*/

            }
            nb--;
        }

    }

    for(int i=0; i<n; i++){
        delete[] V[i];
        delete[] Arg[i];
    }
    delete[] V;
    delete[] Arg;

    //cout<<"on sort de decompose"<<endl;
    //cout<<"V[n-1][0]/V[n-1][NB_VEH-1] = "<<V[n-1][0]/V[n-1][NB_VEH-1]<<endl
    return V[n-1][0]/V[n-1][NB_VEH-1];
}

//Gives the length of the path going from y to x in a tour (y comes before x in tour)
double Heuristics::lengthPath(deque<int> &tour, int y, int x){

    double val = 0;
    int i = 0,j, n = tour.size();
    bool trouve = false, stop = false;

    //cout<<"j'entre dans lengthPath "<<endl;

    if(y!=x){
        while (i<n && !trouve){
            if(tour[i] == y)
                trouve = true;
            else
                i++;
        }
        j = i+1;
        while (j<n && !stop){
            val = val + matrixTruck[tour[j-1]][tour[j]];
            if(tour[j] == x)
                 stop = true;
            else
                j++;
        }
    }

    //cout<<"fin de lengthPath"<<endl;
    return val;
}

//Gives the length of the path from index i to j in tour
double Heuristics::lengthPath2(deque<int> &tour, int i, int j){
    double val = 0;
    while (i<j){
        val = val + matrixTruck[tour[i]][tour[i+1]];
        i = i+1;
    }
    return val;
}


//Filters labels list L by keeping at most SEUIL labels for each numVeh
void Heuristics::filter1(deque <Label2*>& L, int choix){

    deque<Label2*>::iterator it;
    deque<Label2*> tab[NB_VEH];

    //cout<<"*** filter1 choix"<<choix<<" ****"<<endl;

    //we build tab

    for(it=L.begin(); it!=L.end(); ++it){
        tab[(*it)->numVeh-1].push_back(*it);
    }
    /*for(int m=0;m<NB_VEH;m++)
        cout<<"taille de tab["<<m<<"] = "<< tab[m].size()<<endl;
    */


    for(int i=0;i<NB_VEH;i++){
        if(tab[i].size()> SEUIL){
            if (choix == 1)
                sort(tab[i].begin(),tab[i].end(), labelSortCriterion1());
            else
                sort(tab[i].begin(),tab[i].end(), labelSortCriterion2());

            tab[i].resize(SEUIL);
        }
    }

    L.clear();

    for(int i=0;i<NB_VEH;i++){
        for(it=tab[i].begin(); it!=tab[i].end(); ++it)
            L.push_back(*it);
    }
    //cout<<"taille de L apres filtrage = "<<L.size()<<endl;

}

//changes the unit of label l and returns the equivalent class
int Heuristics::transformLabel(Label2* l){

    int v1,v2,v3,q;

    q = ceil(maxi(l->c_gloVeh,l->c1,l->c2)/4);

    v1 = floor(l->c_gloVeh/q);
    v2 = floor(l->c1/q);
    v3 = floor(l->c2/q);

    if(v1==0 && v2==0 && v3==0)
        return 0;
    if(v1==0 && v2==0 && v3==1)
        return 1;
    if(v1==0 && v2==0 && v3==2)
        return 2;
    if(v1==0 && v2==0 && v3==3)
        return 3;
    if(v1==0 && v2==1 && v3==0)
        return 4;
    if(v1==0 && v2==1 && v3==1)
        return 5;
    if(v1==0 && v2==1 && v3==2)
        return 6;
    if(v1==0 && v2==1 && v3==3)
        return 7;
    if(v1==0 && v2==2 && v3==0)
        return 8;
    if(v1==0 && v2==2 && v3==1)
        return 9;
    if(v1==0 && v2==2 && v3==2)
        return 10;
    if(v1==0 && v2==2 && v3==3)
        return 11;
    if(v1==0 && v2==3 && v3==0)
        return 12;
    if(v1==0 && v2==3 && v3==1)
        return 13;
    if(v1==0 && v2==3 && v3==2)
        return 14;
    if(v1==0 && v2==3 && v3==3)
        return 15;

    if(v1==1 && v2==0 && v3==0)
        return 16;
    if(v1==1 && v2==0 && v3==1)
        return 17;
    if(v1==1 && v2==0 && v3==2)
        return 18;
    if(v1==1 && v2==0 && v3==3)
        return 19;
    if(v1==1 && v2==1 && v3==0)
        return 20;
    if(v1==1 && v2==1 && v3==1)
        return 21;
    if(v1==1 && v2==1 && v3==2)
        return 22;
    if(v1==1 && v2==1 && v3==3)
        return 23;
    if(v1==1 && v2==2 && v3==0)
        return 24;
    if(v1==1 && v2==2 && v3==1)
        return 25;
    if(v1==1 && v2==2 && v3==2)
        return 26;
    if(v1==1 && v2==2 && v3==3)
        return 27;
    if(v1==1 && v2==3 && v3==0)
        return 28;
    if(v1==1 && v2==3 && v3==1)
        return 29;
    if(v1==1 && v2==3 && v3==2)
        return 30;
    if(v1==1 && v2==3 && v3==3)
        return 31;

    if(v1==2 && v2==0 && v3==0)
        return 32;
    if(v1==2 && v2==0 && v3==1)
        return 33;
    if(v1==2 && v2==0 && v3==2)
        return 34;
    if(v1==2 && v2==0 && v3==3)
        return 35;
    if(v1==2 && v2==1 && v3==0)
        return 36;
    if(v1==2 && v2==1 && v3==1)
        return 37;
    if(v1==2 && v2==1 && v3==2)
        return 38;
    if(v1==2 && v2==1 && v3==3)
        return 39;
    if(v1==2 && v2==2 && v3==0)
        return 40;
    if(v1==2 && v2==2 && v3==1)
        return 41;
    if(v1==2 && v2==2 && v3==2)
        return 42;
    if(v1==2 && v2==2 && v3==3)
        return 43;
    if(v1==2 && v2==3 && v3==0)
        return 44;
    if(v1==2 && v2==3 && v3==1)
        return 45;
    if(v1==2 && v2==3 && v3==2)
        return 46;
    if(v1==2 && v2==3 && v3==3)
        return 47;

    if(v1==3 && v2==0 && v3==0)
        return 48;
    if(v1==3 && v2==0 && v3==1)
        return 49;
    if(v1==3 && v2==0 && v3==2)
        return 50;
    if(v1==3 && v2==0 && v3==3)
        return 51;
    if(v1==3 && v2==1 && v3==0)
        return 52;
    if(v1==3 && v2==1 && v3==1)
        return 53;
    if(v1==3 && v2==1 && v3==2)
        return 54;
    if(v1==3 && v2==1 && v3==3)
        return 55;
    if(v1==3 && v2==2 && v3==0)
        return 56;
    if(v1==3 && v2==2 && v3==1)
        return 57;
    if(v1==3 && v2==2 && v3==2)
        return 58;
    if(v1==3 && v2==2 && v3==3)
        return 59;
    if(v1==3 && v2==3 && v3==0)
        return 60;
    if(v1==3 && v2==3 && v3==1)
        return 61;
    if(v1==3 && v2==3 && v3==2)
        return 62;
    if(v1==3 && v2==3 && v3==3)
        return 63;

}

//Filters labels list L by keeping at most 64 labels for each numVeh
void Heuristics::filter2(deque <Label2*>& L){

    deque<Label2*>::iterator it;
    int v1,v2,v3,q;

    //cout<<"*** filter2 ****"<<endl;


    const int P = cbrt(SEUIL2);

    //cout<<"P = "<<P<<endl;

    deque<Label2*>classes[P][P][P];
    deque<Label2*> tab[NB_VEH];


    for(it=L.begin(); it!=L.end(); ++it)
        tab[(*it)->numVeh-1].push_back(*it);

    /*for(int m=0;m<NB_VEH;m++)
        cout<<"taille de tab["<<m<<"] = "<< tab[m].size()<<endl;
    */

    for(int i=0;i<NB_VEH;i++){
        if(tab[i].size()> SEUIL2){
            for(int j=0; j<P;j++)
                for(int k=0; k<P;k++)
                    for(int l=0; l<P;l++)
                        classes[j][k][l].clear();

            for(it=tab[i].begin(); it!=tab[i].end(); ++it){

                q = ceil(maxi((*it)->c_gloVeh,(*it)->c1,(*it)->c2)/P)+1;

                //cout<<"q = "<<q<<endl;

                v1 = floor((*it)->c_gloVeh/q);
                v2 = floor((*it)->c1/q);
                v3 = floor((*it)->c2/q);

                //cout<<"v1 = "<<v1<<" , v2 = "<<v2<<" , v3 = "<<v3<<endl;
                classes[v1][v2][v3].push_back(*it);
            }

            //cout<<"on arrive ici"<<endl;
            for(int j=0; j<P;j++)
                for(int k=0; k<P;k++)
                    for(int l=0; l<P;l++){
                        if(!classes[j][k][l].empty())
                            sort(classes[j][k][l].begin(),classes[j][k][l].end(), labelSortCriterion2());
                        //classes[j][k][l].resize(1);
                    }

            tab[i].clear();
            for(int j=0; j<P;j++)
                for(int k=0; k<P;k++)
                    for(int l=0; l<P;l++){
                        if(!classes[j][k][l].empty())
                            tab[i].push_back(classes[j][k][l][0]);
                        if(classes[j][k][l].size() >= 2)
                            tab[i].push_back(classes[j][k][l][1]);
                    }
        }
    }

    L.clear();

    for(int i=0;i<NB_VEH;i++){
        for(it=tab[i].begin(); it!=tab[i].end(); ++it)
            L.push_back(*it);
    }
    //cout<<"taille de L apres filtrage = "<<L.size()<<endl;

}


void Heuristics::applyCrossMove(deque<int> &tour1, deque<int> &tour2, int i, int j){

    //cout<<"On applique un cross move"<<endl;
    deque<int> t1,t2;

    for(int k=0;k<=i;k++)
        t1.push_back(tour1[k]);

    for(int k=j+1;k<tour2.size();k++)
        t1.push_back(tour2[k]);

    for(int k=0;k<=j;k++)
        t2.push_back(tour2[k]);

    for(int k=i+1;k<tour1.size();k++)
        t2.push_back(tour1[k]);


    copy_deque_to_deque(t1,tour1);
    copy_deque_to_deque(t2,tour2);
    /*cout<<"cout t1 = "<<get_truck_cost(t1)<<endl;
    cout<<"cout t2 = "<<get_truck_cost(t2)<<endl;*/

}

void Heuristics::applyExchangeMove1(deque<int> &tour1, deque<int> &tour2, int i, int j){

    //cout<<"On applique un échange entre vehicules"<<endl;
    int c;

    c = tour2[j];
    tour2[j] = tour1[i];
    tour1[i] = c;
}

void Heuristics::applyRelocateMove(deque<int> &tour1, deque<int> &tour2, int i, int j){
    //cout<<"On applique un relocate move véhicule vers vehicule"<<endl;
    std::deque<int>::iterator it;
    /*cout<<"i = "<<i<<endl;
    cout<<"j = "<<j<<endl;
    cout<<"x = "<<drone[j]<<endl;
    cout<<"tour size avant = "<<tour.size()<<endl;
    cout<<"drone size avant = "<<drone.size()<<endl;*/
    tour2.insert (tour2.begin()+j,tour1[i]);
    tour1.erase(tour1.begin()+i);
    /*cout<<"tour size apres = "<<tour.size()<<endl;
    cout<<"drone size apres = "<<drone.size()<<endl;*/
    //best_insertion(tour, x);
}

void Heuristics::applyTransferMove1(deque<int> &tour, deque<int> &drone, int i){
    //cout<<"On applique un transfert vehicule vers drone"<<endl;
    std::deque<int>::iterator it;
    int x = tour[i];
    //cout<<"i = "<<i<<endl;
    //cout<<"x = "<<x<<endl;
    /*for(it=tour.begin(); it!=tour.end();++it)
        cout<<*it<<"->";
    cout<<endl;

    cout<<"ancien cout vehicule = "<<get_truck_cost(tour)<<endl;
    cout<<"ancien size veh"<<tour.size()<<endl;*/
    tour.erase(tour.begin()+i);
    drone.push_back(x);

    /*for(it=tour.begin(); it!=tour.end();++it)
        cout<<*it<<"->";
    cout<<endl;

    cout<<"nouveau cout vehicule = "<<get_truck_cost(tour)<<endl;
    cout<<"nouveau size veh"<<tour.size()<<endl;
    cout<<"cout veh1 = "<<get_truck_cost(trucks_tours[0])<<endl;
    cout<<"cout veh2 = "<<get_truck_cost(trucks_tours[1])<<endl;*/
}

void Heuristics::applyTransferMove2(deque<int> &drone, deque<int> &tour, int i, int j){
    //cout<<"On applique un transfert drone vers vehicule"<<endl;
    std::deque<int>::iterator it;
    /*cout<<"i = "<<i<<endl;
    cout<<"j = "<<j<<endl;
    cout<<"x = "<<drone[j]<<endl;
    cout<<"tour size avant = "<<tour.size()<<endl;
    cout<<"drone size avant = "<<drone.size()<<endl;*/
    tour.insert (tour.begin()+i,drone[j]);
    drone.erase(drone.begin()+j);
    /*cout<<"tour size apres = "<<tour.size()<<endl;
    cout<<"drone size apres = "<<drone.size()<<endl;*/
    //best_insertion(tour, x);
}

void Heuristics::applyExchangeMove2(deque<int> &tour, deque<int> &drone, int i, int j){
    //cout<<"On applique un echange entre vehicule et drone"<<endl;
    //cout<<"i = "<<i<<endl;
    //cout<<"j = "<<j<<endl;
    int c;

    c = tour[i];
    tour[i] = drone[j];
    drone[j] = c;
}


bool Heuristics::crossMove(deque<int> &tour1, deque<int> &tour2, double makespan){
    int i,j,n1 = tour1.size(), n2=tour2.size();
    double u=0,v=0,u1,v1,w1,w2;
    bool success = false;
    deque<int>::iterator it;

    cout<<endl;
    cout<<"Parametre 1 - Vehicule : ";
    for(it = tour1.begin(); it!=tour1.end()-1; ++it)
        cout<<*it<<"->";
    cout<<"0";
    cout<<"    Cout = "<<get_truck_cost(tour1);
    cout<<endl;

    cout<<endl;
    cout<<"Parametre 2 - Vehicule : ";
    for(it = tour2.begin(); it!=tour2.end()-1; ++it)
        cout<< *it<<"->";
    cout<<"0";
    cout<<"   Cout = "<<get_truck_cost(tour2)<<endl;



    i=0;
    j=0;

    while(!success && i<n1-1 && j<n2-1){
        u1 = lengthPath2(tour1, i+1, n1-1);
        //get_truck_cost(tour1) - lengthPath2(tour1, 0, i) - matrixTruck[tour1[i]][tour1[i+1]];
        v1 = lengthPath2(tour2, j+1, n2-1);
        //get_truck_cost(tour2) - lengthPath2(tour2, 0, j) - matrixTruck[tour1[j]][tour1[j+1]];
        w1 = lengthPath2(tour1, 0, i) + matrixTruck[tour1[i]][tour2[j+1]] + v1;
        w2 = lengthPath2(tour2, 0, j) + matrixTruck[tour2[j]][tour1[i+1]] + u1;

        cout<<endl;
        cout<<"Test cross move entre l'indice i = "<<i<<" du vehicule(param1) et l'indice j= "<<j<<" du vehicule(param2)"<<endl;

        if(w1<makespan && w2 < makespan && !double_equals(w1,makespan) && !double_equals(w2,makespan)){
            //cout<<"w1 = "<<w1<<endl;
            //cout<<"w2 = "<<w2<<endl;
            success = true;
            cout<<"SUCCES!!! On pplique le cross move"<<endl;
            applyCrossMove(tour1,tour2,i,j);
        }else{
            if(w1<makespan && !double_equals(w1,makespan))
                i = i+1;
            else{
                if(w2<makespan && !double_equals(w2,makespan))
                    j = j+1;
                else{
                    i=i+1;
                    j=j+1;
                }
            }
            cout<<"Pas d'amelioration du makespan"<<endl;
        }
    }
    return success;
}

bool Heuristics::exchangeMove1(deque<int> &tour1, deque<int> &tour2, double makespan){
    bool success = false;
    double cost1, cost2, val1, val2;
    int i=1, n=tour1.size(), n2 = tour2.size(),j=1;
    deque<int>::iterator it;

    cout<<endl;
    cout<<"Parametre 1 - Vehicule : ";
    for(it = tour1.begin(); it!=tour1.end()-1; ++it)
        cout<<*it<<"->";
    cout<<"0";
    cout<<"    Cout = "<<get_truck_cost(tour1);
    cout<<endl;

    cout<<endl;
    cout<<"Parametre 2 - Vehicule : ";
    for(it = tour2.begin(); it!=tour2.end()-1; ++it)
        cout<< *it<<"->";
    cout<<"0";
    cout<<"   Cout = "<<get_truck_cost(tour2)<<endl;

    cost1 = get_truck_cost(tour1);
    cost2 = get_truck_cost(tour2);

    while(!success && i<n-1){
        j = 1;
        while(j<n2-1 && !success){
            cout<<endl;
            cout<<"Test echange client_Veh(param1) "<< tour1[i]<< " et client_Veh(param2) "<< tour2[j]<<endl;

            val1 = cost1 - matrixTruck[tour1[i-1]][tour1[i]] - matrixTruck[tour1[i]][tour1[i+1]]
               + matrixTruck[tour1[i-1]][tour2[j]] + matrixTruck[tour2[j]][tour1[i+1]];
            val2 = cost2 - matrixTruck[tour2[j-1]][tour2[j]] - matrixTruck[tour2[j]][tour2[j+1]]
               + matrixTruck[tour2[j-1]][tour1[i]] + matrixTruck[tour1[i]][tour2[j+1]];

            cout<<"Cout vehicule (param1) si echange = "<<val1<<endl;
            cout<<"Cout vehicule (param2) si echange = "<<val2<<endl;
            if((val1 < makespan)&&(val2 < makespan)&&
            !double_equals(val1,makespan) && !double_equals(val2,makespan)){
                    cout<<"SUCCES!!! On applique l'ehange"<<endl;
                    success = true;
                    applyExchangeMove1(tour1,tour2,i,j);
                }else
                    cout<<"Pas d'amelioration du makespan"<<endl;
                j++;
        }
        i++;
    }
    return success;
}

bool Heuristics::relocateMove(deque<int> &tour1, deque<int> &tour2, double makespan){
    bool success = false;
    double cost1, cost2, val1, val2;
    int i=1, n=tour1.size(), n2 = tour2.size(),j=1;
    deque<int>::iterator it;

    cout<<endl;
    cout<<"Parametre 1 - Vehicule : ";
    for(it = tour1.begin(); it!=tour1.end()-1; ++it)
        cout<<*it<<"->";
    cout<<"0";
    cout<<"    Cout = "<<get_truck_cost(tour1);
    cout<<endl;

    cout<<endl;
    cout<<"Parametre 2 - Vehicule : ";
    for(it = tour2.begin(); it!=tour2.end()-1; ++it)
        cout<< *it<<"->";
    cout<<"0";
    cout<<"   Cout = "<<get_truck_cost(tour2)<<endl;

    cost1 = get_truck_cost(tour1);
    cost2 = get_truck_cost(tour2);
    //cout<<"n = "<<n<<" et n2 = "<<n2<<endl;

    while(!success && i<n-1){
        //cout<<"on entre ici"<<endl;
        //cout<<"i = "<<i<<endl;
        /*if(is_drone_eligible(tour1[i]))
            cout<<"Il est drone eligible"<<endl;
        else
            cout<<"Il n'est pas drone eligible"<<endl;*/
        j = 1;
        while(j<=n2-1 && !success){
            cout<<endl;
            cout<<"Test relocate du client vehicule (param1) "<< tour1[i]<<" a la position "<<j<< " du vehicule (param2)"<<endl;
            val1 = cost1 - matrixTruck[tour1[i-1]][tour1[i]] - matrixTruck[tour1[i]][tour1[i+1]]
                        + matrixTruck[tour1[i-1]][tour1[i+1]];

            val2 = cost2 - matrixTruck[tour2[j-1]][tour2[j]] + matrixTruck[tour2[j-1]][tour1[i]]
                         + matrixTruck[tour1[i]][tour2[j]];

            //cout<<"val1 = "<< val1<<endl;
            //cout<<"val2 = "<< val2<<endl;

            cout<<"Cout vehicule (param1) si relocate = "<<val1<<endl;
            cout<<"Cout vehicule (param2) si relocate = "<<val2<<endl;

            if(val1 < makespan && val2 < makespan &&
           !double_equals(val1,makespan) && !double_equals(val2,makespan)){
                success = true;
                cout<<"SUCCES!!! On applique le relocate"<<endl;
                applyRelocateMove(tour1, tour2, i, j);
            }else
                cout<<"Pas d'amelioration du makespan"<<endl;

            j++;
        }
        i++;
    }
    return success;
}

bool Heuristics::transferMove1(deque<int> &tour, deque<int> &drone, double makespan){
    int i=1,n=tour.size();
    double droneCost = get_drone_cost(drone), tourCost = get_truck_cost(tour), val1,val2;
    bool success = false;
    deque<int>::iterator it;

    cout<<endl;
    cout<<"Parametre 1 - Vehicule : ";
    for(it = tour.begin(); it!=tour.end()-1; ++it)
        cout<<*it<<"->";
    cout<<"0";
    cout<<"    Cout = "<<get_truck_cost(tour);
    cout<<endl;

    cout<<endl;
    cout<<"Parametre 2 - Drone : ";
    for(it = drone.begin(); it!=drone.end(); ++it)
        cout<< *it<<";";
    cout<<"   Cout = "<<get_drone_cost(drone)<<endl;

    while(!success && i<n-1){

        val1 = matrixDrone[0][tour[i]]*2 + droneCost;
        val2 = tourCost - matrixTruck[tour[i-1]][tour[i]] - matrixTruck[tour[i]][tour[i+1]] + matrixTruck[tour[i-1]][tour[i+1]];
        cout<<endl;
        cout<<"Test transfert client "<< tour[i]<<endl;
        if(is_drone_eligible(tour[i])){
            cout<<"Il est drone eligible. Super! Le tranfert est possible"<<endl;
            cout<<"Cout drone si transfert = "<< val1<<endl;
            cout<<"Cout vehicule si transfert = "<< val2<<endl;

        }
        else
            cout<<"Il n'est pas drone eligible. Dommage! Le transfert n'est pas possible"<<endl;


        if(is_drone_eligible(tour[i]) && (val1 < makespan) && (val2 < makespan) &&
           !double_equals(val1,makespan) && !double_equals(val2,makespan)){
            //cout<<"i == "<<i<<endl;
            //cout<<"tourCost = "<<tourCost<<endl;
            //cout<<"verif1 = "<< tourCost - matrixTruck[tour[i-1]][tour[i]] - matrixTruck[tour[i]][tour[i+1]] + matrixTruck[tour[i-1]][tour[i+1]]<<endl;
            //cout<<"verif2 = "<< matrixDrone[0][tour[i]]*2 + droneCost<<endl;

            success = true;
            cout<<"SUCCES!!! On applique le transfert"<<endl;
            applyTransferMove1(tour,drone,i);
        }else{
            if (is_drone_eligible(tour[i]))
                cout<<"Pas d'amelioration du makespan"<<endl;
            i=i+1;
        }
    }
    return success;
}

bool Heuristics::transferMove2(deque<int> &drone, deque<int> &tour, double makespan){
    int i=1,n=tour.size(), n2 =drone.size(), j;
    double droneCost = get_drone_cost(drone), tourCost = get_truck_cost(tour), val;
    bool success = false;
    deque<int>::iterator it;

    cout<<endl;
    cout<<"Parametre 1 - Drone : ";
    for(it = drone.begin(); it!=drone.end(); ++it)
        cout<<*it<<";";
    cout<<"    Cout = "<<get_drone_cost(drone);
    cout<<endl;

    cout<<endl;
    cout<<"Parametre 2 - Vehicule : ";
    for(it = tour.begin(); it!=tour.end()-1; ++it)
        cout<< *it<<"->";
    cout<<"0";
    cout<<"   Cout = "<<get_truck_cost(tour)<<endl;

    while(!success && i<n-1){

        j=0;
        while(!success && j<n2){
            cout<<endl;
            cout<<"Test transfert client_Dro "<<drone[j]<<" a la position "<<i<<" du vehicule"<<endl;
            val = tourCost + matrixTruck[tour[i]][drone[j]] + matrixTruck[drone[j]][tour[i+1]] -
               matrixTruck[tour[i]][tour[i+1]];

            cout<<"Cout vehicule si transfert  = "<< val<<endl;
            if(val < makespan && !double_equals(val,makespan)){
                success = true;
                cout<<"SUCCES!!! On applique le transfert"<<endl;
                applyTransferMove2(drone, tour, i+1, j);
            }else
                cout<<"Pas d'amelioration du makespan"<<endl;
            j++;
        }
        i++;
    }
    return success;
}


bool Heuristics::exchangeMove2(deque<int> &tour, deque<int> &drone, double makespan){
    bool success = false;
    int i=1, n=tour.size(), n2 = drone.size(),j;
    double droneCost = get_drone_cost(drone), tourCost = get_truck_cost(tour), w1, w2, val2;
    deque<int>::iterator it;

    cout<<endl;
    cout<<"Parametre 1 - Vehicule : ";
    for(it = tour.begin(); it!=tour.end()-1; ++it)
        cout<<*it<<"->";
    cout<<"0";
    cout<<"    Cout = "<<get_truck_cost(tour);
    cout<<endl;

    cout<<endl;
    cout<<"Parametre 2 - Drone : ";
    for(it = drone.begin(); it!=drone.end(); ++it)
        cout<< *it<<";";
    cout<<"   Cout = "<<get_drone_cost(drone)<<endl;
    cout<<endl;

    while(!success && i<n-1){
        //cout<<"i = "<<i<<endl;
        j = 0;
        if(is_drone_eligible(tour[i])){
            cout<<endl;
            cout<<"Le client "<<tour[i]<< " est drone eligible. Super! L'echange est possible"<<endl;
            while(j<n2 && !success){
                cout<<endl;
                cout<<"Test echange client_Veh "<< tour[i]<< " et client_Dro "<< drone[j]<<endl;
                w1 = matrixTruck[tour[i-1]][drone[j]] + matrixTruck[drone[j]][tour[i+1]];
                w2 = matrixTruck[tour[i-1]][tour[i]] + matrixTruck[tour[i]][tour[i+1]];
                val2 = droneCost + (matrixDrone[0][tour[i]]*2) - (matrixDrone[0][drone[j]]*2);
                cout<<"Avant echange : Cout "<<tour[i-1]<<"->"<<tour[i]<<"->"<<tour[i+1]<<" = "<<w2<<endl;
                cout<<"Si echange : Cout "<<tour[i-1]<<"->"<<drone[j]<<"->"<<tour[i+1]<<" = "<<w1<<endl;
                if(w1 < w2 && !double_equals(w1,w2)){
                    cout<<"Cout drone si echange = "<<val2<<endl;
                    if(val2 < makespan && !double_equals(val2,makespan)){
                        success = true;
                        cout<<"SUCCES!!! On applique l'echange"<<endl;
                        applyExchangeMove2(tour,drone,i,j);
                    }else
                        cout<<"Cout drone si echange depasse le makespan. Dommage!"<<endl;
                }else
                    cout<<"Pas d'amelioration du cout vehicule si echange"<<endl;
                j++;
            }
        }else{
            cout<<endl;
            cout<<"Le client "<<tour[i]<<" n'est pas drone eligible. Dommage! L'echange n'est pas possible"<<endl;
        }
        i++;
    }

    return success;

}


bool Heuristics::transfer1_echange(deque<int> &tour, deque<int> &drone, double makespan){
    int j=0,n=tour.size(), n2=drone.size(), i=1;
    double droneCost = get_drone_cost(drone), val;
    double tourCost = get_truck_cost(tour);
    bool success = false;



    while(!success && i<n-1){
        j=0;
        while(!success && j<n2){
            val = tourCost + matrixTruck[tour[i]][drone[j]] + matrixTruck[drone[j]][tour[i+1]] -
               matrixTruck[tour[i]][tour[i+1]];
            //cout<<"val = "<<val<<endl;
            if(val < makespan){
                    success = true;
                    applyTransferMove2(drone,tour, i+1, j);
            }else{
                if(is_drone_eligible(tour[i]) &&
                   makespan - (matrixDrone[0][drone[j]]*2)+
                   (matrixDrone[0][tour[i]]*2) < makespan &&
                   tourCost + matrixTruck[tour[i-1]][drone[j]]+matrixTruck[drone[j]][tour[i+1]]
                   - matrixTruck[tour[i-1]][tour[i]] - matrixTruck[tour[i]][tour[i+1]] < makespan){
                        success = true;
                        applyExchangeMove2(tour,drone,i,j);
                   }
            }
            j++;
        }
        i++;
    }

    return success;

}

void Heuristics::sortCost(deque < deque<int> > &tours, deque < deque<int> > &drones){
    double costDrone[NB_DRONE];
    double costVeh[NB_VEH];

    deque<int> n;
    int taille = tours.size();
    for(int i=taille-1; i>0; i--){
        for(int j=0; j<i; j++){
            if ( get_truck_cost(tours[j])< get_truck_cost(tours[j+1])){
                copy_deque_to_deque(tours[j], n);
                copy_deque_to_deque(tours[j+1], tours[j]);
                copy_deque_to_deque(n, tours[j+1]);
            }
        }
    }

    taille = drones.size();
    for(int i=taille-1; i>0; i--){
        for(int j=0; j<i; j++){
            if ( get_drone_cost(drones[j])< get_drone_cost(drones[j+1])){
                copy_deque_to_deque(drones[j], n);
                copy_deque_to_deque(drones[j+1], drones[j]);
                copy_deque_to_deque(n, drones[j+1]);
            }
        }
    }
}

Label* Heuristics::split(deque <Node*> tour, int iteration){

    bool stop, added=true;
    int length = tour.size(), j, choosenBsup;
    double S, BSup1, BSup2, BSup3, BSup;
    double* BInf1, *BInf2;
    Label *l, *best_label;
    deque<Label*>::iterator it, it2;  //An iterator on labels list
    deque<Node*>::iterator it3;  //An iterator on Node list

    nbGenLab = 0;
    nbDelLab = 0;


    //VERIFICATION
    /*cout<<" ============ Voici le tour ============="<<endl;
    cout<<endl;
    for(int i=0; i<length; i++)
        cout<<tour[i]->get_id()<<";";
    cout<<endl;*/


    /******** Upper bounds and lower bounds computation *******/

    //Upper bounds computation
    BSup1 = bSup1(tour); //Alain
    BSup2 = bSup2(tour); //Dominique

    //cout<<"bsup1 = "<<BSup1<<" et bsup2 = "<<BSup2<<endl;
    if (iteration > 1)//if it is not the first iteration of the split (there is already a recor solution)
        BSup3 = max(recor_label->c1,recor_label->c2);

    //We determine which upper bound we will use (this info is useful for the case where labels list of destination depot is empty)
    if (min(BSup1,BSup2) == BSup1){
        BSup = BSup1;
        choosenBsup = 1;
    }else{
        if(min(BSup1,BSup2) == BSup2){
            BSup = BSup2;
            choosenBsup = 2;
        }else{
            BSup = BSup2;
            choosenBsup = 2;
        }
    }

    //cout<<"BSup1 = "<<BSup1<<endl;
    //cout<<"BSup2 = "<<BSup2<<endl;

    if(iteration>1){
        if(min(BSup,BSup3) == BSup3){
            BSup = BSup3;
            choosenBsup = 3;
        }
    }


    //Lower bounds computation
    BInf1 = bInf(tour,BSup,1); //in the sens of the vehicle
    BInf2 = bInf(tour,BSup,2); //in the sens of the sum

    /******** Application of BELLMAN procedure ********/

    //we ensure that the label list of each node in tour is empty
    for(int i=0; i<length; i++){
        std::for_each(tour[i]->labels.begin(), tour[i]->labels.end(), Delete());
        tour[i]->labels.clear();
    }

    l = new Label();
    l->current_node_id = tour[0]->get_id();
    l->c1 = 0;
    l->c2 = 0;
    l->father = NULL;

    nbGenLab++; //we increment the number of labels generated

    tour[0]->labels.push_back(l); //we but label (0,0) in the label list of tour[0]

    for(int i=1; i<length; i++){
        //cout<<endl;
        //cout<<"************************   noeud = "<<i<<"    ***********************"<<endl;

        stop = false;
        S = 0.; //will contain the cost related to the drone
        j = i-1; //j is the predecessor of i in tour

        while((stop == false)&&(j>=0)){
            //cout<<"j = "<<j<<endl;
            //if(!tour[j]->labels.empty())
            for(it = tour[j]->labels.begin(); it!=tour[j]->labels.end(); ++it){
                l = new Label();
                l->c1 = (*it)->c1 + matrixTruck[tour[j]->get_id()][tour[i]->get_id()];
                l->c2 = (*it)->c2 + S/NB_DRONE;
                l->current_node_id = tour[i]->get_id();
                l->father = (*it);

                nbGenLab ++;
                /*cout<<"je genere un label"<<endl;
                cout<<"l est ("<<l->c1<<","<<l->c2<<")"<<endl;*/

                added = true; //will verify if created label is added

                if((l->c2 < BSup)&&(l->c1+BInf1[i] < BSup) && (l->c1+l->c2+BInf2[i] < 2*BSup)){//l is not pruned by filtering rules using upper and lower bounds
                    //We try to add l to tour[i]->labels list
                    //cout<<"l n'est pa supprime par les bornes"<<endl;
                    if (!tour[i]->labels.empty()){
                        it2 = tour[i]->labels.begin();
                        while(it2 <tour[i]->labels.end()){
                            if(dominance_test(l,(*it2))){ //l is dominated by the label pointed by it2
                                 //cout<<"j'entre ici l est domine"<<endl;
                                 added = false;
                                 delete l;
                                 nbDelLab++; //we increment the number of deleted labels
                                 break;
                            }else{
                                 if(dominance_test((*it2),l)){//true if the label pointed by it is dominated by l
                                     //cout<<"j'entre ici l domine"<<endl;
                                     delete (*it2);
                                     it2 = tour[i]->labels.erase(it2);
                                     nbDelLab++; //we increment the number of deleted labels
                                 }else
                                    it2++;
                            }
                        }
                    }
                    if(added) //we add l to tour[i]->labels list
                        tour[i]->labels.push_back(l);
                }else{//l is pruned by filtering rules using upper and lower bounds
                    //cout<<"l est supprime par les bornes"<<endl;
                    delete l;
                    nbDelLab++; //we increment the number of deleted labels
                }
            }

            if ((tour[j]->get_id() == 0) || !is_drone_eligible(tour[j]->get_id())) //if j is not drone eligible or j is the origin depot
                stop = true;
            else{
                S = S + (matrixDrone[0][tour[j]->get_id()])*2;
                j = j-1; //we go to the next predecessor
            }
        }

        //VERIFICATION
        /*cout<<"liste des labels du noeud "<<i<<endl;
        for(it2=tour[i]->labels.begin(); it2<tour[i]->labels.end(); it2++)
            cout<<"("<<(*it2)->c1<<","<<(*it2)->c2<<");";
        cout<<endl;*/


    }

    if(!tour[length-1]->labels.empty()){//if the label list of the destination depot is not empty

        //we find the best label in destination depot labels list
        best_label = find_best_label(tour[length-1]->labels);

        //we backtrack to find the truck tour and the drone list
        truck_tour.clear();
        truck_tour.push_front(0);//we put the destination depot id in the truck_solution_tour
        Label * father;
        father = best_label->father;

        while(father != NULL){
            truck_tour.push_front(father->current_node_id);//we add
            father = father->father;
        }

        drone_list.clear();
        for(it3 = all_customers.begin(); it3!=all_customers.end(); ++it3){
            if(!is_in_truck_tour((*it3)->get_id(), truck_tour))
                drone_list.push_back((*it3)->get_id());
        }
    }else{ //the destination depot labels list is empty so we use the solution associated to bSup
        //cout <<"!!!!!!!!!!!!! J'utilise la solution BSup !!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        //cout<<"borne sup choisie "<<choosenBsup<<endl;
        if (choosenBsup == 1){
            copy_deque_to_deque(bsup1_truck_tour, truck_tour);
            copy_deque_to_deque(bsup1_drone_list, drone_list);
            best_label = bsup1_label;
        }else{
            if(choosenBsup == 2){
                copy_deque_to_deque(bsup2_truck_tour, truck_tour);
                copy_deque_to_deque(bsup2_drone_list, drone_list);
                best_label = bsup2_label;
            }else{
                copy_deque_to_deque(best_truck_tour,truck_tour);
                copy_deque_to_deque(best_drone_list,drone_list);
                best_label = recor_label;
            }
        }
    }

    //free memory

    bsup1_truck_tour.clear();
    bsup1_drone_list.clear();
    //delete bsup1_label;
    bsup2_truck_tour.clear();
    bsup2_drone_list.clear();
    //delete bsup2_label;

    delete [] BInf1;
    delete [] BInf2;


    /*//VERIFICATION
    cout<<"********* Truck_tour sortie de bellmann *********"<<endl;
    int taille = truck_tour.size();
    for(int i=0; i<taille; i++)
        cout<<truck_tour[i]<<";";
    cout<<endl;

    cout<<"*********Drone_list sortie de bellmann*********"<<endl;
    taille = drone_list.size();
    for(int i=0; i<taille; i++)
        cout<<drone_list[i]<<";";
    cout<<endl;
    */
    //cout<<"le meilleur label à la sortie du split est ("<<best_label->c1<<","<<best_label->c2<<")"<<endl;
    //cout<<"nbre de label genere = "<<nbGenLab<<endl;


    return best_label;

}

Label2* Heuristics::split2(deque <Node*> tour, int iteration, int limitLab){

    bool stop, added=true, test;
    int length = tour.size(), j, destDepotId = all_customers.size(),choosenBsup;
    double S, BSup1, BSup2,BSup3, BSup, val;
    double *BInf2;
    //deque<Label2*> tab[NB_VEH];
    double **V;
    deque <int> tourN;
    Label2 *l, *best_label;
    deque<Label2*> bestLabels;
    deque<Label2*>::iterator it, it2;  //An iterator on labels2 list
    deque<Node*>::iterator it3;  //An iterator on Node list
    deque<int>::iterator it4;  //An iterator on int list
    deque < deque<int> >::iterator it5;

    nbGenLab = 0;
    nbDelLab = 0;

    cout<<endl;
    cout<<"################ Decoding : Split procedure ################"<<endl;
    cout<<endl;
    //VERIFICATION
    /*cout<<" ============ Voici le tour ============="<<endl;
    cout<<endl;
    for(int i=0; i<length; i++)
        cout<<tour[i]->get_id()<<"->";
    cout<<endl;*/


    /******** Upper bounds and lower bounds computation *******/

    for(it3=tour.begin(); it3!=tour.end()-1;++it3)
        tourN.push_back((*it3)->get_id());

    tourN.push_back(destDepotId);

    //Upper bounds computation
    BSup1 = bSup3(tourN); //Alain
    BSup2 = bSup4(tourN); //Laurent

    //cout<<"BSup Alain = "<<BSup1<<endl;
    //cout<<"BSup Laurent = "<<BSup2<<endl;

    BSup = max(BSup1,BSup2);
    //cout<<"bsup1 = "<<BSup1<<" et bsup2 = "<<BSup2<<endl;
    if (iteration > 1)//if it is not the first iteration of the split (there is already a recor solution)
        BSup3 = max(recor_label->c1,recor_label->c2);

    //We determine which upper bound we will use (this info is useful for the case where labels list of destination depot is empty)
    if (min(BSup1,BSup2) == BSup1){
        BSup = BSup1;
        choosenBsup = 1;
    }else{
        if(min(BSup1,BSup2) == BSup2){
            BSup = BSup2;
            choosenBsup = 2;
        }else{
            BSup = BSup2;
            choosenBsup = 2;
        }
    }

    cout<<"BSup = "<<BSup<<endl;
    //cout<<"BSup2 = "<<BSup2<<endl;

    if(iteration>1){
        if(min(BSup,BSup3) == BSup3){
            BSup = BSup3;
            choosenBsup = 3;
        }
    }


    if(limitLab !=5){

        //Lower bounds computation
        BInf2 = bInf(tour,BSup,2); //in the sens of the sum

        V = preprocess(tourN);


        /******** Application of BELLMAN procedure ********/

        //we ensure that the label list of each node in tour is empty
        for(int i=0; i<length; i++){
            std::for_each(tour[i]->labels2.begin(), tour[i]->labels2.end(), Delete());
            tour[i]->labels2.clear();
        }

        l = new Label2();
        l->current_node_id = tour[0]->get_id();
        l->c_gloVeh = 0;
        l->c1 = 0;
        l->c2 = 0;
        l->numVeh = 1;
        l->father = NULL;

        nbGenLab++; //we increment the number of labels generated

        val = bInf(tour,0,V,l);
        //if((maxi(l->c_gloVeh,l->c1,l->c2)< BSup)&&(val< BSup) &&
         //              (l->c1+l->c2+BInf2[0] < (NB_VEH-l->numVeh+2)*BSup))
            tour[0]->labels2.push_back(l); //we but label (0,0) in the label list of tour[0]
        /*else{
            delete l;
            nbDelLab++; //we increment the number of deleted labels
        }*/

        for(int i=1; i<length; i++){
            //cout<<endl;
            //cout<<"************************   noeud = "<<i<<"    ***********************"<<endl;

            //for(int m=0;m<NB_VEH;m++)
            //    tab[m].clear();

            stop = false;
            S = 0.; //will contain the cost related to the drone
            j = i-1; //j is the predecessor of i in tour

            while((stop == false)&&(j>=0)){
                //cout<<"j = "<<j<<endl;
                //if(!tour[j]->labels.empty())
                for(it = tour[j]->labels2.begin(); it!=tour[j]->labels2.end(); ++it){

                    //new label by keeping the same current vehicle
                    l = new Label2();
                    l->c_gloVeh = (*it)->c_gloVeh;
                    l->c1 = (*it)->c1 + matrixTruck[tour[j]->get_id()][tour[i]->get_id()];
                    l->c2 = (*it)->c2 + S/NB_DRONE;
                    l->numVeh = (*it)->numVeh;
                    l->current_node_id = tour[i]->get_id();
                    l->father = (*it);

                    nbGenLab ++;
                    //cout<<"je genere un label"<<endl;
                    //cout<<"l est ("<<l->c_gloVeh<<","<<l->c1<<","<<l->c2<<","<<l->numVeh")"<<endl;

                    added = true; //will verify if created label is added
                    //cout<<"on arrive ici"<<endl;
                    val = bInf(tour,i,V,l);
                    //cout<<"Binf(l) = "<<val;
                    if((maxi(l->c_gloVeh,l->c1+matrixTruck[0][tour[i]->get_id()],l->c2)< BSup)&&(val< BSup) &&
                       (l->c1+l->c2+BInf2[i] < (NB_VEH-l->numVeh+2)*BSup)){//l is not pruned by filtering rules using upper and lower bounds
                        //We try to add l to tour[i]->labels list
                        //cout<<"l n'est pa supprime par les bornes"<<endl;
                        if (!tour[i]->labels2.empty()){
                            it2 = tour[i]->labels2.begin();
                            while(it2 <tour[i]->labels2.end()){
                                if(dominance_test2(l,(*it2),tour[i]->get_id())){ //l is dominated by the label pointed by it2
                                     //cout<<"j'entre ici l est domine"<<endl;
                                     added = false;
                                     delete l;
                                     nbDelLab++; //we increment the number of deleted labels
                                     break;
                                }else{
                                     if(dominance_test2((*it2),l,tour[i]->get_id())){//true if the label pointed by it is dominated by l
                                         //cout<<"j'entre ici l domine"<<endl;
                                         delete (*it2);
                                         it2 = tour[i]->labels2.erase(it2);
                                         nbDelLab++; //we increment the number of deleted labels
                                     }else
                                        it2++;
                                }
                            }
                        }
                        if(added){ //we add l to tour[i]->labels list
                            tour[i]->labels2.push_back(l);
                            //cout<<"BInf ("<<l->c_gloVeh<<","<<l->c1<<","<<l->c2<<","<<l->numVeh<<") = "<<val<<endl;

                            //tab[l->numVeh-1].push_back(l);
                        }
                    }else{//l is pruned by filtering rules using upper and lower bounds
                        //cout<<"l est supprime par les bornes"<<endl;
                        delete l;
                        nbDelLab++; //we increment the number of deleted labels
                    }

                    if(((*it)->c1 + matrixTruck[tour[j]->get_id()][tour[i]->get_id()] +
                       matrixTruck[tour[i]->get_id()][0] > max((*it)->c_gloVeh, (*it)->c2))&&
                       (*it)->numVeh < NB_VEH && tour[j]->get_id() != 0 && i != length-1){

                            //new label by changing vehicle
                            l = new Label2();
                            l->c_gloVeh = max((*it)->c_gloVeh, (*it)->c1 + matrixTruck[tour[j]->get_id()][0]);
                            l->c1 = matrixTruck[0][tour[i]->get_id()];
                            l->c2 = (*it)->c2 + S/NB_DRONE;
                            l->numVeh = (*it)->numVeh + 1;
                            l->current_node_id = tour[i]->get_id();
                            l->father = (*it);

                            nbGenLab ++;
                            val = bInf(tour,i,V,l);
                            //cout<<"je genere un label"<<endl;
                            //cout<<"l est ("<<l->c_gloVeh<<","<<l->c1<<","<<l->c2<<","<<l->numVeh")"<<endl;

                            added = true; //will verify if created label is added

                            if((maxi(l->c_gloVeh,l->c1+matrixTruck[0][tour[i]->get_id()],l->c2)< BSup)&&(val< BSup) &&
                                (l->c1+l->c2+BInf2[i] < (NB_VEH-l->numVeh+2)*BSup)){//l is not pruned by filtering rules using upper and lower bounds
                                //We try to add l to tour[i]->labels list
                                //cout<<"l n'est pa supprime par les bornes"<<endl;
                                if (!tour[i]->labels2.empty()){
                                    it2 = tour[i]->labels2.begin();
                                    while(it2 <tour[i]->labels2.end()){
                                        if(dominance_test2(l,(*it2),tour[i]->get_id())){ //l is dominated by the label pointed by it2
                                             //cout<<"j'entre ici l est domine"<<endl;
                                             added = false;
                                             delete l;
                                             nbDelLab++; //we increment the number of deleted labels
                                             break;
                                        }else{
                                             if(dominance_test2((*it2),l,tour[i]->get_id())){//true if the label pointed by it is dominated by l
                                                 //cout<<"j'entre ici l domine"<<endl;
                                                 delete (*it2);
                                                 it2 = tour[i]->labels2.erase(it2);
                                                 nbDelLab++; //we increment the number of deleted labels
                                             }else
                                                it2++;
                                        }
                                    }
                                }
                                if(added){ //we add l to tour[i]->labels list
                                    tour[i]->labels2.push_back(l);
                                    //cout<<"BInf ("<<l->c_gloVeh<<","<<l->c1<<","<<l->c2<<","<<l->numVeh<<") = "<<val<<endl;

                                    //tab[l->numVeh-1].push_back(l);
                                }
                            }else{//l is pruned by filtering rules using upper and lower bounds
                                //cout<<"l est supprime par les bornes"<<endl;
                                delete l;
                                nbDelLab++; //we increment the number of deleted labels
                            }

                    }

                }

                if ((tour[j]->get_id() == 0) || !is_drone_eligible(tour[j]->get_id())) //if j is not drone eligible or j is the origin depot
                    stop = true;
                else{
                    S = S + (matrixDrone[0][tour[j]->get_id()])*2;
                    j = j-1; //we go to the next predecessor
                }
            }

            //check if we have to call filter function
            if (limitLab == 1)
                filter1(tour[i]->labels2, 1);
            else{
                if(limitLab == 2)
                    filter1(tour[i]->labels2, 2);
                else{
                    if(limitLab == 3)
                        filter2(tour[i]->labels2);
                }
            }

            //cout<<"taille de L avant filtrage = "<<tour[i]->labels2.size()<<endl;

            /*for(int m=0;m<NB_VEH;m++){
                if(tab[m].size()> SEUIL){
                    call_filter = true;
                    //break;
                }
                cout<<"taille de tab["<<m<<"] = "<< tab[m].size()<<endl;
            }*/

            //if(call_filter){
                //filter2(tour[i]->labels2);
            //}

            //VERIFICATION
            /*cout<<endl;
            cout<<"liste des labels du noeud "<<i<<endl;
            for(it2=tour[i]->labels2.begin(); it2<tour[i]->labels2.end(); it2++)
                cout<<"("<<(*it2)->c_gloVeh<<","<<(*it2)->c1<<","<<(*it2)->c2<<","<<(*it2)->numVeh<<");";
            cout<<endl;*/

        }
        /*//VERIFICATION
        cout<<endl;
        cout<<"liste des labels du noeud "<<length<<endl;
        for(it2=tour[length-1]->labels2.begin(); it2<tour[length-1]->labels2.end(); it2++)
            cout<<"("<<(*it2)->c_gloVeh<<","<<(*it2)->c1<<","<<(*it2)->c2<<","<<(*it2)->numVeh<<");";
        cout<<endl;
        */

        if(!tour[length-1]->labels2.empty()){//if the label list of the destination depot is not empty

            Label2 * father;

            /*if(NB_DRONE > 1){
                double tourCost[NB_VEH]; //cost of each vehicle tour
                bool tourOpt[NB_VEH]; //indicates if a tour has already been optimized
                bool optimize;
                double worstTourCost,worst,cost;
                Label *l1;
                //we find the best label in destination depot labels list
                bestLabels.clear();
                bestLabels = find_best_label3(tour[length-1]->labels2);
                int compt = bestLabels.size(), c,worstTourPos,bestIndex;

                //deque<int> deques[NB_VEH]; //table of list (each list will contain the set of customer id associated to the given vehicle)
                deque<int> tabs[compt][NB_VEH];
                deque <int> drone_lists[compt];
                deque <deque<int> > drone_assignments[compt];
                deque<Label*> realLabels;

                for(int i=0; i<compt; i++)
                    for(int j=0; j<NB_VEH; j++)
                        tabs[i][j].clear();//we make sure each vehicle tour is clear (for each label in bestLabels)

                for(int i=0; i<compt; i++)
                    for(int j=0; j<NB_VEH; j++)
                        tabs[i][j].push_front(0);//we put the destination depot id in each truck tour

                realLabels.clear();

                c = 0;
                while(c < compt){
                    best_label = bestLabels[c];

                    //we backtrack to find the trucks tour and the drone list

                    //vehicles
                    trucks_tours.clear();

                    father = best_label->father;

                    while(father != NULL && father->current_node_id != 0){

                        tabs[c][father->numVeh-1].push_front(father->current_node_id);//we add
                        father = father->father;
                    }

                    for(int i=0; i<NB_VEH; i++)
                        tabs[c][i].push_front(0);//we put the origin depot id in each truck tour

                    //transfer deques to trucks_tours
                    trucks_tours.clear();
                    for(int i=0; i<NB_VEH; i++){
                        trucks_tours.push_back(tabs[c][i]);
                    }

                    //drone

                    drone_lists[c].clear();//we make sure the drone list is clear

                    for(it3 = all_customers.begin(); it3!=all_customers.end(); ++it3){
                        test = false;
                        for(int i=0; i<NB_VEH; i++){
                            if(is_in_truck_tour((*it3)->get_id(), tabs[c][i])){
                                test = true;
                                break;
                            }
                        }
                        if(!test)
                            drone_lists[c].push_back((*it3)->get_id());
                    }

                    copy_deque_to_deque(drone_lists[c],drone_list);


                    //VERIFICATION
                    cout<<"********* Trucks_tours sortie de bellmann *********"<<endl;
                    //int taille = trucks_tours.size();
                    for(int i=0; i<NB_VEH; i++){
                        cout<<endl;
                        cout<<"** Tour du vehicule "<<i<<" **"<<endl;
                        for(it4 = trucks_tours[i].begin(); it4!=trucks_tours[i].end(); ++it4)
                            cout<<*it4<<";";
                        cout<<endl;
                    }

                    cout<<"*********Drone_list sortie de bellmann*********"<<endl;
                    int taille = drone_list.size();
                    for(int i=0; i<taille; i++)
                        cout<<drone_list[i]<<";";
                    cout<<endl;


                    /********* Anticipation of the second phase *************/

                    //vehicles optimization

                    /*for(int i=0; i<NB_VEH; i++)
                        tourCost[i] = get_truck_cost(trucks_tours[i]);

                    for(int i=0; i<NB_VEH; i++)
                        tourOpt[i] = false;

                    optimize = true;
                    while (optimize){

                        //we search the worst vehicle tour
                        worstTourPos = 0;
                        worstTourCost = tourCost[0];
                        for(int i=1; i<NB_VEH; i++){
                            if(tourCost[i]>worstTourCost){
                                worstTourCost = tourCost[i];
                                worstTourPos = i;
                            }
                        }

                        if(!tourOpt[worstTourPos] && trucks_tours[worstTourPos].size()>3){
                            trucks_tours[worstTourPos].pop_back();
                            create_inputFile_forLkh(trucks_tours[worstTourPos]);
                            cost = run_lkh(trucks_tours[worstTourPos]);//puts the optimized tour in truck_tour

                            trucks_tours[worstTourPos].clear();
                            copy_deque_to_deque(truck_tour, trucks_tours[worstTourPos]);
                            tourCost[worstTourPos] = cost;
                            tourOpt[worstTourPos] = true;
                        }else
                            optimize = false;
                    }
                    //we search for the worst cost in tourCost
                    worst = tourCost[0];
                    for(int i=1; i<NB_VEH; i++){
                        if(tourCost[i]>worst){
                            worst = tourCost[i];
                        }
                    }


                    //VERIFICATION
                    /*cout<<"Avant"<<endl;
                    for(int i=0; i<NB_VEH; i++){
                        for(it4=tabs[c][i].begin(); it4!=tabs[c][i].end();++it4)
                            cout<<*it4<<"->";
                        cout<<endl;
                    }*/

                    //we put the optimized tours in tabs
                    /*for(int i=0; i<NB_VEH; i++){

                        copy_deque_to_deque(trucks_tours[i], tabs[c][i]);
                    }*/

                    //VERIFICATION
                    /*cout<<"Apres"<<endl;
                    for(int i=0; i<NB_VEH; i++){
                        for(it4=tabs[c][i].begin(); it4!=tabs[c][i].end();++it4)
                            cout<<*it4<<"->";
                        cout<<endl;
                    }
                    //cout<<"worst apres optimisation = "<<worst<<endl;

                    //we apply LPT to see the real cost
                    //cout<<"cout LPT = "<<lpt_heuristic()<<endl;

                    l1 = new Label();
                    l1->c1 = worst;
                    l1->c2 = lpt_heuristic();

                    realLabels.push_back(l1);
                    copy_dequetab_to_dequetab(drone_assignment, drone_assignments[c]);
                    c++;
                    */
                //}


                //verification
                /*cout<<"Voici les valeur de la solution reelle après optimisation"<<endl;
                for(int i=0; i< compt; i++)
                    cout<<"("<<realLabels[i]->c1<<","<<realLabels[i]->c2<<")"<<endl;
                */
                //we search for the index of the best label in realLabels

                /*


                Label *bestLab = realLabels[0];
                bestIndex = 0;
                double diff = realLabels[0]->c2 - bestLabels[0]->c2;
                for(int i=1; i< compt; i++)
                    if(compare_label(realLabels[i],bestLab)){
                    //if((realLabels[i]->c2 - bestLabels[i]->c2) < diff){
                        bestLab = realLabels[i];
                        bestIndex = i;
                        diff = realLabels[i]->c2 - bestLabels[i]->c2;
                    }

                //we chose the best solution
                //deque< deque<int> > result = transformed(tabs[bestIndex]);
                //cout<<"taille de result = "<<result.size()<<endl;
                //cout<<"bestIndex = "<<bestIndex<<endl;

                //for(it5 = trucks_tours.begin(); it5!=trucks_tours.end(); ++it5)
                //    (*it5).clear();
                //trucks_tours.clear();

                //VERIFICATION
                /*cout<<"Avant"<<endl;
                for(it5=trucks_tours.begin(); it5!=trucks_tours.end();++it5){
                    for(it4=(*it5).begin(); it4!=(*it5).end();++it4)
                        cout<<*it4<<"->";
                    cout<<endl;
                }


                for(int j=0;j<NB_VEH;j++){
                    copy_deque_to_deque(tabs[bestIndex][j],trucks_tours[j]);
                }

                //VERIFICATION
                cout<<"Apres"<<endl;
                for(it5=trucks_tours.begin(); it5!=trucks_tours.end();++it5){
                    for(it4=(*it5).begin(); it4!=(*it5).end();++it4)
                        cout<<*it4<<"->";
                    cout<<endl;
                }


                //cout<<"taille de trucks_tours = "<<trucks_tours.size()<<endl;
                //copy_dequetab_to_dequetab(result, trucks_tours);
                copy_deque_to_deque(drone_lists[bestIndex], drone_list);
                //copy_dequetab_to_dequetab(drone_assignments[bestIndex],drone_assignment);

                //VERIFICATION
                cout<<"drone list"<<endl;
                for(it4=drone_list.begin(); it4!=drone_list.end(); ++it4)
                    cout<<*it4<<";";
                cout<<endl;

                //best_label->c1 = bestLab->c1;
                //best_label->c2 = bestLab->c2;


                best_label = bestLabels[bestIndex];
            }else{*/





                //we find the best label in destination depot labels list
                best_label = find_best_label2(tour[length-1]->labels2);

                //we backtrack to find the trucks tour and the drone list
                trucks_tours.clear();
                deque<int> deques[NB_VEH]; //table of list (each list will contain the set of customer id associated to the given vehicle)

                for(int i=0; i<NB_VEH; i++)
                    deques[i].push_front(0);//we put the destination depot id in each truck tour

                father = best_label->father;

                while(father != NULL && father->current_node_id != 0){

                    deques[father->numVeh-1].push_front(father->current_node_id);//we add
                    father = father->father;
                }

                for(int i=0; i<NB_VEH; i++)
                    deques[i].push_front(0);//we put the origin depot id in each truck tour

                //transfer deques to trucks_tours
                for(int i=0; i<NB_VEH; i++){
                    trucks_tours.push_back(deques[i]);
                }



                drone_list.clear();
                for(it3 = all_customers.begin(); it3!=all_customers.end(); ++it3){
                    test = false;
                    for(int i=0; i<NB_VEH; i++){
                        if(is_in_truck_tour((*it3)->get_id(), deques[i])){
                            test = true;
                            break;
                        }
                    }
                    if(!test)
                        drone_list.push_back((*it3)->get_id());
                }

            //}
        }else{ //the destination depot labels list is empty so we use the solution associated to bSup
            //cout <<"!!!!!!!!!!!!! J'utilise la solution BSup !!!!!!!!!!!!!!!!!!!!!!!"<<endl;
            //cout<<"borne sup choisie "<<choosenBsup<<endl;
            if (choosenBsup == 1){
                copy_dequetab_to_dequetab(bsup3_trucks_tours, trucks_tours);
                copy_deque_to_deque(bsup3_drone_list, drone_list);
                best_label->c_gloVeh = bsup3_label->c1;
                best_label->c1 = bsup3_label->c1;
                best_label->c2 = bsup3_label->c2;
            }else{
                if(choosenBsup == 2){
                    copy_dequetab_to_dequetab(bsup4_trucks_tours, trucks_tours);
                    copy_deque_to_deque(bsup4_drone_list, drone_list);
                    best_label->c_gloVeh = bsup4_label->c1;
                    best_label->c1 = bsup4_label->c1;
                    best_label->c2 = bsup4_label->c2;
                }else{
                    copy_dequetab_to_dequetab(best_trucks_tours,trucks_tours);
                    copy_deque_to_deque(best_drone_list,drone_list);
                    best_label->c_gloVeh = recor_label->c1;
                    best_label->c1 = recor_label->c1;
                    best_label->c2 = recor_label->c2;
                }
            }
        }
    }else{
        //cout <<"!!!!!!!!!!!!! Pas de split !!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        //cout<<"borne sup choisie "<<choosenBsup<<endl;
        if (choosenBsup == 1){
            copy_dequetab_to_dequetab(bsup3_trucks_tours, trucks_tours);
            copy_deque_to_deque(bsup3_drone_list, drone_list);
            best_label->c_gloVeh = bsup3_label->c1;
            best_label->c1 = bsup3_label->c1;
            best_label->c2 = bsup3_label->c2;
        }else{
            if(choosenBsup == 2){
                copy_dequetab_to_dequetab(bsup4_trucks_tours, trucks_tours);
                copy_deque_to_deque(bsup4_drone_list, drone_list);
                best_label->c_gloVeh = bsup4_label->c1;
                best_label->c1 = bsup4_label->c1;
                best_label->c2 = bsup4_label->c2;
            }else{
                copy_dequetab_to_dequetab(best_trucks_tours,trucks_tours);
                copy_deque_to_deque(best_drone_list,drone_list);
                best_label->c_gloVeh = recor_label->c1;
                best_label->c1 = recor_label->c1;
                best_label->c2 = recor_label->c2;
            }
        }
    }
    //free memory

    /*bsup1_truck_tour.clear();
    bsup1_drone_list.clear();
    bsup2_truck_tour.clear();
    bsup2_drone_list.clear();
    //delete bsup2_label;

    delete [] BInf1;
    delete [] BInf2;
    */


    //VERIFICATION
    cout<<endl;
    cout<<" ------ Solution a la sortie du SPLIT ------"<<endl;
    //int taille = trucks_tours.size();
    for(int i=0; i<NB_VEH; i++){
        cout<<endl;
        cout<<"Tour du vehicule "<<i+1<<" : ";
        for(it4 = trucks_tours[i].begin(); it4!=trucks_tours[i].end()-1; ++it4)
            cout<<*it4<<"->";
        cout<<"0"<<endl;
        cout<<"Cout vehicule "<<i+1<<" = "<<get_truck_cost(trucks_tours[i]);
        cout<<endl;
    }

    cout<<endl;
    cout<<"Liste des clients drone : ";
    int taille = drone_list.size();
    for(int i=0; i<taille-1; i++)
        cout<<drone_list[i]<<";";
    cout<<drone_list[taille-1];
    cout<<endl;

    /*cout<<"Solution courante = ("<<best_label->c_gloVeh<<","<<best_label->c1<<
    ","<<best_label->c2<<","<<best_label->numVeh<<")"<<endl;*/

    cout<<endl;
    cout<<"currentSol = ("<<max(best_label->c_gloVeh,best_label->c1)<<
    ","<<best_label->c2<<")"<<endl;

    //cout<<"nbre de label genere = "<<nbGenLab<<endl;


    return best_label;

}


















//Longest Processing Time Heuristic
double Heuristics::lpt_heuristic(){

    deque<int> deques[NB_DRONE]; //table of list (each list will contain the set of customer id associated to the given drone)
    double endingTimes[NB_DRONE]; //indicates the ending times for each drone
    int j;
    double cost;
    deque<int>::iterator it;
    deque<deque <int> >::iterator it2;

    drone_assignment.clear();

    //we initialize all the ending times to zero
    for(int i=0; i<NB_DRONE; i++){
        endingTimes[i] = 0;
    }

    sortDroneDeque(); //sort drone_list in a decreasing order of fly back and forth to the depot

    //cout<<"drone list in LPT"<<endl;
    for (it = drone_list.begin(); it!=drone_list.end(); ++it){
         //cout<<"client "<<*it<<"; cout = "<<matrixDrone[0][(*it)]*2<<endl;
        //we search for the first free drone
        j = findSmallestDroneTime(endingTimes);
        deques[j].push_back(*it);
        endingTimes[j] = endingTimes[j] + (matrixDrone[0][*it])*2;
    }
    //cout<<endl;

    for(int i=0; i<NB_DRONE; i++){
        if(!deques[i].empty())
            drone_assignment.push_back(deques[i]);
    }

    //we determine the cost
    cost = endingTimes[0];
    //cout<<"cout drone 1 = "<<cost<<endl;
    for(int i=1; i<NB_DRONE; i++){
        //cout<<"cout drone "<<i+1<<" = "<<endingTimes[i]<<endl;
        if(endingTimes[i] > cost)
            cost = endingTimes[i];
    }

    //VERIFICATION
    /*for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
        cout<<"Liste des clients du drone i"<<endl;
        for(it = (*it2).begin(); it!=(*it2).end(); ++it){
            cout<<*it<<";";
        }
        cout<<endl;
    }*/


    //delete endingTimes;
    //delete deques;

    return cost;
}

//Run the Two-step heuristic
void Heuristics::run_heuristic(string path, string file_n)
{
    clock_t tStart = clock(); //to have the execution time at the end
    Label * current_label, *curSol;
    int nb_iteration, nbSplitCall = 0, averageNbGenLab=0, averageNbDelLab=0, nb_restart = 0;
    //int averageGlobalNbGenLab=0, averageGlobalNbDelLab=0, averageGlobalGapDrone = 0;
    double cost, averageGapDrone = 0., maxGapDrone = 0., droneCostFromSplit, percentDelLab=0.;
    deque<Node*>::iterator it;  //An iterator on Node list
    deque<deque <int> >::iterator it2;
    deque<int>::iterator it3, it4;  //An iterator on int list
    deque<int> listeN;
    tsp *giant_tsp;
    string path1 = path+"/result.csv", path2,str;
    const char *filename = path1.c_str();
    bool improve;

    final_recor_label = new Label();

    current_label = new Label();
    recor_label = new Label();

    while((double)(clock() - tStart)/CLOCKS_PER_SEC <= TIME_LIMIT){

       //cout<<" ******* NB REDEMARRAGE "<<nb_restart<<" *******"<<endl;
       //cout<<endl;

        /******* Step 1 : build a giant TSP *******/

        //tsp with 2-opt
        giant_tsp = new tsp(all_customers); //creates a tsp object
        cost = giant_tsp->run_tsp(nb_restart); //the result is found in solution deque
        //cout <<"cout du TSP = "<<cost<<endl;

        for(it3=giant_tsp->solution.begin(); it3!=giant_tsp->solution.end(); ++it3){
            tour.push_back(new Node(*find_node_by_id(*it3)));
            //cout<<(*it3)<<";";
        }
        //cout<<endl;


        //tsp with LKH
        /*listeN.clear();
        for(it=all_customers.begin(); it!=all_customers.end(); ++it)
            listeN.push_back((*it)->get_id());
        create_inputFile_forLkh(listeN);
        cost = run_lkh(listeN);
        cout <<"cout de LKH = "<<cost<<endl;*/

        //we add the depot at the end of tour
        tour[0]->set_drone(1); //the origin depot node is imposed to the truck
        tour.push_back(new Node(*tour[0]));


        //we set tour as the best_truck_tour found so far
        best_truck_tour.clear();
        for(it = tour.begin(); it!=tour.end(); ++it){
            best_truck_tour.push_back((*it)->get_id());
        }

        recor_label->c1 = cost;
        recor_label->c2 = 0;

        nb_iteration = 1;
        improve = true;

        if(!droneEligible.empty())
        while(improve){
            /*cout<<endl;
            cout<<" ******* Iteration "<<nb_iteration<<" *******"<<endl;
            cout<<endl;*/

            /******* Step 2 : apply Split procedure on tour  *******/

            curSol = split(tour,nb_iteration);
            nbSplitCall ++; //counts the number of split calls
            averageNbGenLab = averageNbGenLab + nbGenLab;
            averageNbDelLab = averageNbDelLab + nbDelLab;

            current_label->c1 = curSol->c1;
            current_label->c2 = curSol->c2;
            droneCostFromSplit = curSol->c2;

            //cout<<"cout vehicule avant optimisation = "<<curSol->c1<<endl;

            /******* Step 3 : optimize truck_tour and run longest processing time heuristic *******/

            truck_tour.pop_back();
            create_inputFile_forLkh(truck_tour);
            cost = run_lkh(truck_tour);
            //cout<<"cout vehicule apres optimisation LKH = "<<cost<<endl;
            //cost = amelioration();//Apply amelioration of truck_tour with 2-opt moves and returns the cost after improvement
            current_label->c1 = cost;

            cost = lpt_heuristic();
            current_label->c2 = cost;
            //cout<<"cost = "<<cost<<endl;
            //cout<<"c1 = "<<current_label->c1<<" ; c2 = "<<current_label->c2<<endl;
            if(cost > 0){
                averageGapDrone = averageGapDrone + fabs(100.*((droneCostFromSplit-cost)/cost));
                if(fabs(100.*((droneCostFromSplit-cost)/cost))> maxGapDrone)
                   maxGapDrone = fabs(100.*((droneCostFromSplit-cost)/cost));
            }
            if(compare_label(current_label,recor_label)){//True if current_label is better than recor_label

                recor_label->c1 = current_label->c1;
                recor_label->c2 = current_label->c2;

                copy_deque_to_deque(truck_tour,best_truck_tour);
                copy_deque_to_deque(drone_list,best_drone_list);
                copy_dequetab_to_dequetab(drone_assignment, best_drone_assignment);

                /******* Step 4 : best insertion (to have a new giant_tsp) *******/

                for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
                    for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                        best_insertion(truck_tour,*it3);
                    }
                }

                //we delete the old tour
                for(it = tour.begin(); it!=tour.end(); ++it)
                    delete *it;
                tour.clear();

                //we build the new giant tour
                for(it3 = truck_tour.begin(); it3!=truck_tour.end(); ++it3){
                    tour.push_back(new Node(*find_node_by_id(*it3)));
                }

            }else{//the current solution is not better than the recor solution
                //cout<<"STOP!!!"<<endl;
                improve = false;

                //we delete tour to free the memory
                for(it = tour.begin(); it!=tour.end(); ++it)
                    delete *it;
                tour.clear();
            }

            truck_tour.clear();
            drone_list.clear();
            for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2)
                (*it2).clear();
            drone_assignment.clear();

            nb_iteration++;

        }

        //we delete the tour
        std::for_each(tour.begin(), tour.end(), Delete());
        tour.clear();


        if(nb_restart == 0){

            final_recor_label->c1 = recor_label->c1;
            final_recor_label->c2 = recor_label->c2;

            copy_deque_to_deque(best_truck_tour, final_truck_tour);
            copy_deque_to_deque(best_drone_list, final_drone_list);
            copy_dequetab_to_dequetab(best_drone_assignment, final_drone_assignment);

        }else{
            //cout<<"j'entre ici"<<endl;
            if(compare_label(recor_label,final_recor_label)){//final_recor_label is dominated by recor_label
                final_recor_label->c1 = recor_label->c1;
                final_recor_label->c2 = recor_label->c2;

                copy_deque_to_deque(best_truck_tour, final_truck_tour);
                copy_deque_to_deque(best_drone_list, final_drone_list);
                copy_dequetab_to_dequetab(best_drone_assignment, final_drone_assignment);
            }
        }

        best_truck_tour.clear();
        best_drone_list.clear();
        for(it2 = best_drone_assignment.begin(); it2!=best_drone_assignment.end(); ++it2)
            (*it2).clear();
        best_drone_assignment.clear();

        //cout<<"final recor est ("<<final_recor_label->c1<<","<<final_recor_label->c2<<")"<<endl;

        //averageNbGenLab = averageNbGenLab/(nb_iteration-1);
        //averageNbDelLab = averageNbDelLab/(nb_iteration-1);
        //averageGapDrone = averageGapDrone/(nb_iteration-1);

        //averageGlobalNbGenLab = averageGlobalNbGenLab + averageNbGenLab;
        //averageGlobalNbDelLab = averageGlobalNbDelLab + averageNbDelLab;
        //averageGlobalGapDrone = averageGlobalGapDrone + averageGapDrone;
        //system("pause");
        nb_restart++;
        //if(droneEligible.empty())
        delete giant_tsp;
    }

    if(averageNbGenLab != 0)
        percentDelLab = ((double)averageNbDelLab/ (double)averageNbGenLab)*100.;

    if(nbSplitCall != 0){

    //cout<<"averageGapDrone = "<<averageGapDrone<<endl;

    averageNbGenLab = averageNbGenLab/nbSplitCall;
    //averageNbDelLab = averageNbDelLab/nbSplitCall;
    averageGapDrone = averageGapDrone/nbSplitCall;
    //averageGlobalNbGenLab = averageGlobalNbGenLab/(nb_restart);
    //averageGlobalNbDelLab = averageGlobalNbDelLab/(nb_restart);
    //averageGlobalGapDrone = averageGlobalGapDrone/(nb_restart);
    nbSplitCall = nbSplitCall/(nb_restart);
    }

    /*cout<<"liste des clients servis par le drone"<<endl;
    for(it3 = final_drone_list.begin(); it3!=final_drone_list.end(); ++it3){
        cout<<*it3<<endl;
    }*/

    cout<<"maxGapDrone = "<<maxGapDrone<<endl;
    cout<<"avgGapDrone = "<<averageGapDrone<<endl;
    //We write the solution in the result file
    //write_result(filename,file_n.c_str(),final_recor_label, final_truck_tour.size()-2,final_drone_list.size(), (double)(clock() - tStart)/CLOCKS_PER_SEC, nb_restart, nbSplitCall, averageNbGenLab, percentDelLab, averageGapDrone);

    delete current_label;
    delete recor_label;
    delete final_recor_label;
}


//Run the Two-step heuristic
void Heuristics::run_heuristic2(string path, string file_n)
{
    clock_t tStart = clock(); //to have the execution time at the end
    Label2 *curSol;
    Label * current_label;
    int nb_iteration, nbSplitCall = 0, averageNbGenLab=0, averageNbDelLab=0, nb_restart = 0;
    int worstTourPos, posInsert, nb, destDepotId = all_customers.size(), cmpt, i, j;
    //int averageGlobalNbGenLab=0, averageGlobalNbDelLab=0, averageGlobalGapDrone = 0;
    double cost, averageGapDrone = 0., maxGapDrone = 0., droneCostFromSplit, percentDelLab=0., worstTourCost,worstDrone, makespan;
    double timeSplit=0, timeOptimize=0, timeLocalSearch=0;
    deque<Node*>::iterator it;  //An iterator on Node list
    deque<deque <int> >::iterator it2;
    deque<int>::iterator it3, it4;  //An iterator on int list
    deque<int> listeN, liste;
    deque<int> values;
    double tourCost[NB_VEH]; //cost of each vehicle tour
    bool tourOpt[NB_VEH]; //indicates if a tour has already been optimized
    tsp *giant_tsp;
    ostringstream s1, s2, s3,s4;
    s1 << NB_VEH;
    s2 << NB_DRONE;
    s3 << SPEED;
    s4 << LIMIT_LAB_METHOD;
    string path1 = path+"/result_"+s1.str()+"_"+s2.str()+"_"+s3.str()+"_"+s4.str()+".csv", path2,str;
    const char *filename = path1.c_str();
    bool improve, optimize, check, success=false;

    final_recor_label = new Label();

    current_label = new Label();
    recor_label = new Label();

    using namespace std::chrono;

    while((double)(clock() - tStart)/CLOCKS_PER_SEC <= TIME_LIMIT){

       cout<<endl;
       cout<<" ******* Demarrage numero "<<nb_restart<<" *******"<<endl;
       cout<<endl;

        /******* Initialization : Build a giant TSP *******/

        //tsp with 2-opt
        giant_tsp = new tsp(all_customers); //creates a tsp object
        cout<<endl;
        cout<<"******** Tour géant TSP initial *********"<<endl;
        cout<<endl;

        cost = giant_tsp->run_tsp(time(NULL) + nb_restart); //the result is found in solution deque
        cout <<"cout du TSP = "<<cost<<endl;

        for(it3=giant_tsp->solution.begin(); it3!=giant_tsp->solution.end(); ++it3){
            tour.push_back(new Node(*find_node_by_id(*it3)));
            //cout<<(*it3)<<";";
        }
        //cout<<endl;


        //tsp with LKH (when %DE=0%)
        /*listeN.clear();
        for(it=all_customers.begin(); it!=all_customers.end(); ++it)
            listeN.push_back((*it)->get_id());
        create_inputFile_forLkh(listeN);
        cost = run_lkh(listeN);
        cout <<"cout de LKH = "<<cost<<endl;*/

        /*
        //tour for instanceArticle
        for(it=all_customers.begin(); it!=all_customers.end(); ++it)
            tour.push_back(new Node(*(*it)));
        */


        //we add the depot at the end of tour
        tour[0]->set_drone(1); //the origin depot node is imposed to the truck
        tour.push_back(new Node(*tour[0]));

        int taille = tour.size();
        tour[taille-1]->set_id(destDepotId);



        /*
        //TEST
        listeN.clear();
        for(it=tour.begin(); it!=tour.end(); ++it)
            listeN.push_back((*it)->get_id());

        listeN.pop_back();
        listeN.push_back(listeN.size());
        //double coefAcc = decompose(listeN, 0);
        double Bsup = bSup4(listeN);
        cout<<"Bsup4 = "<<Bsup<<endl;*/

        //we set tour as the best_trucks_tours found so far
        best_trucks_tours.clear();
        liste.clear();

        for(it = tour.begin(); it!=tour.end(); ++it)
            liste.push_back((*it)->get_id());
        best_trucks_tours.push_back(liste);


        recor_label->c1 = get_truck_cost(liste);
        recor_label->c2 = 0;

        //cout<<"recor_label = ("<<recor_label->c1<<","<<recor_label->c2<<")"<<endl;

        nb_iteration = 1;
        improve = true;
        cmpt = 0;

        if(!droneEligible.empty())
        while(cmpt <= SEUIL_CMPT){
            /*cout<<endl;
            cout<<" ******* Iteration "<<nb_iteration<<" *******"<<endl;
            cout<<endl;*/
            /******* Split Procedure  *******/

             // Get starting timepoint
            auto start = high_resolution_clock::now();

            curSol = split2(tour,nb_iteration,LIMIT_LAB_METHOD);
            // Get ending timepoint
            auto stop = high_resolution_clock::now();

            std::chrono::duration<double> diff = stop-start;
            timeSplit = timeSplit + diff.count();

            std::cout << "Temps d'execution Split "<< diff.count() << " s\n";
            cout<<endl;

            nbSplitCall ++; //counts the number of split calls
            averageNbGenLab = averageNbGenLab + nbGenLab;
            averageNbDelLab = averageNbDelLab + nbDelLab;

            current_label->c1 = max(curSol->c_gloVeh,curSol->c1) ;
            //cout<<"current_label->c1 = "<<current_label->c1<<endl;
            current_label->c2 = curSol->c2;
            droneCostFromSplit = curSol->c2;

            //cout<<"current_label = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;
            //cout<<"cout vehicule avant optimisation = "<<curSol->c1<<endl;



            /*cout<<"********* Trucks_tours Avant OPTIMISATION *********"<<endl;
            for(int i=0; i<NB_VEH; i++){
                cout<<"le cout du tour "<<i+1<<" est "<<get_truck_cost(trucks_tours[i])<<endl;
            }*/

            /******* Route reoptimization + Drone assignment (LKH + PMS) *******/

            cout<<endl;
            cout<<"******** Optimisation LKH + PMS *********"<<endl;
            cout<<endl;


            start = high_resolution_clock::now();

            //if(NB_DRONE == 1){
                int len = trucks_tours.size();
                for(int i=0; i<len; i++)
                    tourCost[i] = get_truck_cost(trucks_tours[i]);

              /*  for(int i=0; i<NB_VEH; i++)
                    tourOpt[i] = false;

                optimize = true;
                while (optimize){

                    //we search the worst vehicle tour=
                    worstTourPos = 0;
                    worstTourCost = tourCost[0];
                    for(int i=1; i<NB_VEH; i++){
                        if(tourCost[i]>worstTourCost){
                            worstTourCost = tourCost[i];
                            worstTourPos = i;
                        }
                    }

                    if(!tourOpt[worstTourPos] && trucks_tours[worstTourPos].size()>3){
                        trucks_tours[worstTourPos].pop_back();
                        create_inputFile_forLkh(trucks_tours[worstTourPos]);
                        cost = run_lkh(trucks_tours[worstTourPos]);

                        trucks_tours[worstTourPos].clear();
                        copy_deque_to_deque(truck_tour, trucks_tours[worstTourPos]);
                        tourCost[worstTourPos] = cost;
                        tourOpt[worstTourPos] = true;
                    }else
                        optimize = false;
                }
                */


                //cout<<"len = "<<len<<endl;
                for(int i=0; i<len; i++){

                    /*cout<<"tour "<<i+1<<endl;
                    for(it4 = trucks_tours[i].begin(); it4!=trucks_tours[i].end(); ++it4)
                        cout<<*it4<<";";
                    cout<<endl;*/

                    if(trucks_tours[i].size() > 3){
                        trucks_tours[i].pop_back();
                        create_inputFile_forLkh(trucks_tours[i]);
                        cost = run_lkh(trucks_tours[i]);
                        //cout<<"cool"<<endl;
                        trucks_tours[i].clear();
                        copy_deque_to_deque(truck_tour, trucks_tours[i]);
                        tourCost[i] = cost;
                    }
                }

                //we search the worst vehicle tour
                worstTourPos = 0;
                worstTourCost = tourCost[0];
                for(int i=1; i<len; i++){
                    if(tourCost[i]>worstTourCost){
                        worstTourCost = tourCost[i];
                        worstTourPos = i;
                    }
                }

                /*//Verification
                cout<<"********* Trucks_tours Apres OPTIMISATION *********"<<endl;
                //int taille = trucks_tours.size();
                for(int i=0; i<NB_VEH; i++){
                    cout<<endl;
                    cout<<"** Tour du vehicule "<<i<<" **"<<endl;
                    for(it4 = trucks_tours[i].begin(); it4!=trucks_tours[i].end(); ++it4)
                        cout<<*it4<<";";
                    cout<<endl;
                }
                for(int i=0; i<NB_VEH; i++){
                    cout<<"le cout du tour "<<i+1<<" est "<<get_truck_cost(trucks_tours[i])<<endl;
                }


                for(int i=0; i<NB_VEH; i++)
                    if(tourOpt[i])
                        cout<<"Opt_"<<i+1<<" = Vrai";
                    else
                        cout<<"Opt_"<<i+1<<" = Faux";
                cout<<endl;

                cout<<"worstTourCost = "<<worstTourCost<<endl;
                */
                //This is the current solution
                current_label->c1 = worstTourCost;


                cost = lpt_heuristic();
                //cout<<"cout apres LPT (run_heuristic) = "<<cost<<endl;
                //cout<<"la taille de drone_assignment apres LPT est "<<drone_assignment.size()<<endl;
                current_label->c2 = cost;

            //}

            cout<<"current_label after LKH + PMS = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;



            /*cout<<"********* Trucks_tours Apres OPTIMISATION *********"<<endl;
            //int taille = trucks_tours.size();
            for(int i=0; i<NB_VEH; i++){
                cout<<endl;
                cout<<"** Tour du vehicule "<<i<<" **"<<endl;
                for(it4 = trucks_tours[i].begin(); it4!=trucks_tours[i].end(); ++it4)
                    cout<<*it4<<";";
                cout<<endl;
            }
            cout<<"***** liste drones Apres OPTIMISATION *****"<<endl;
            for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
                for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                    cout<< *it3<<",";
                }
                cout<<endl;
            }*/
            stop = high_resolution_clock::now();
            diff = stop-start;
            timeOptimize = timeOptimize + diff.count();

            std::cout << "Temps d'execution Optimize "<< diff.count() << " s\n";
            cout<<endl;


            start = high_resolution_clock::now();

            /******** Local search : moves operations *********/
            cout<<endl;
            cout<<"******** Recherche Locale *********"<<endl;
            cout<<endl;

            sortCost(trucks_tours,drone_assignment);
            worstTourCost = get_truck_cost(trucks_tours[0]);
            worstDrone = get_drone_cost(drone_assignment[0]);
            makespan = max(worstTourCost, worstDrone);

            cout<<"makespan = "<<makespan<<endl;

            if(worstTourCost > worstDrone){ //makespan is given by a vehicle
                cout<<"makespan par un vehicule"<<endl;
                success = false;
                j=drone_assignment.size()-1;
                while(j>=0 && !success){
                    success = transferMove1(trucks_tours[0],drone_assignment[j],makespan);
                    j--;
                }

                if(!success){
                    cout<<"Echec de tous les TRANSFER MOVE Veh -> Dro" <<endl;
                    j=drone_assignment.size()-1;
                    while(j>=0 && !success){
                        success = exchangeMove2(trucks_tours[0],drone_assignment[j],makespan);
                        j--;
                    }
                    if(!success){
                        cout<<"Echec de tous les EXCHANGE MOVE Veh-Dro"<<endl;
                        i=trucks_tours.size()-1;
                        while(i>1 && !success){
                            success = relocateMove(trucks_tours[0],trucks_tours[i],makespan);
                            i--;
                        }

                        if(!success){
                            cout<<"Echec de tous les RELOCATE MOVE"<<endl;
                            i=trucks_tours.size()-1;
                            while(i>1 && !success){
                                success = exchangeMove1(trucks_tours[0],trucks_tours[i],makespan);
                                i--;
                            }
                            if(!success){
                                cout<<"Echec de tous les EXCHANGE MOVE Veh-Veh"<<endl;
                                i=trucks_tours.size()-1;
                                while(i>1 && !success){
                                    success = crossMove(trucks_tours[0],trucks_tours[i],makespan);
                                    i--;
                                }
                            }
                        }
                    }
                }
                if(success){
                    //cout<<"Wouppiiiiiiiiiiiii Success !!!! On continu"<<endl;
                    current_label->c1 = get_worst_veh_cost(trucks_tours);
                    current_label->c2 = get_worst_drone_cost(drone_assignment);
                    cout<<"Nouvelle Solution Courante = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;
                    cout<<endl;
                }

            }else{//makespan is given by a drone
                cout<<"makespan par un drone"<<endl;
                success = false;
                i=trucks_tours.size()-1;
                while(i>=0 && !success){
                    success = transferMove2(drone_assignment[0], trucks_tours[i], makespan);
                    i--;
                }
                if(!success){
                    cout<<"Echec de tous les TRANSFER MOVE Dro -> Veh"<<endl;
                    i=trucks_tours.size()-1;
                    while(i>=0 && !success){
                        success = exchangeMove2(trucks_tours[i], drone_assignment[0], makespan);
                        i--;
                    }
                    if(!success){
                        cout<<"Echec de tous les EXCHANGE MOVE Veh-Dro"<<endl;
                    }
                }
                if(success){
                    //cout<<"Wouppiiiiiiiiiiiii Success !!!! On continu "<<endl;
                    current_label->c1 = get_worst_veh_cost(trucks_tours);
                    current_label->c2 = get_worst_drone_cost(drone_assignment);
                    cout<<"Nouvelle Solution Courante = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;
                    cout<<endl;
                }
            }


            stop = high_resolution_clock::now();
            diff = stop-start;
            timeLocalSearch = timeLocalSearch + diff.count();

            std::cout << "Temps d'execution Local Search"<< diff.count() << " s\n";
            cout<<endl;








            /******** test moves operations *********/
/*
            sortCost(trucks_tours,drone_assignment);
            makespan = max(worstTourCost,cost);
            if(worstTourCost > cost){ //makespan is given by a vehicle
                cout<<"makespan par un vehicule"<<endl;
                success = false;
                int i=trucks_tours.size()-1;
                while(i>0 && !success){
                    success = crossMove(trucks_tours[0],trucks_tours[i],makespan);
                    i--;
                }
                if(!success){
                    int j=drone_assignment.size()-1;
                    while(j>=0 && !success){
                        success = transferMove1(trucks_tours[0],drone_assignment[j],makespan);
                        j--;
                    }
                    if(!success){
                        int j=drone_assignment.size()-1;
                        while(j>=0 && !success){
                            success = exchangeMove2(trucks_tours[0],drone_assignment[j],makespan);
                            j--;
                        }
                    }
                }
                if(success){
                    cout<<"Wouppiiiiiiiiiiiii Success !!!! "<<endl;
                    current_label->c1 = get_worst_veh_cost(trucks_tours);
                    current_label->c2 = get_worst_drone_cost(drone_assignment);
                    cout<<"After Moves current_label = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;
                }

            }else{//makespan is given by a drone
                cout<<"makespan par un drone"<<endl;
                success = false;
                int i=trucks_tours.size()-1;
                while(i>=0 && !success){
                    success = transfer1_echange(trucks_tours[i],drone_assignment[0], makespan);
                    i--;
                }
                if(success){
                    cout<<"Wouppiiiiiiiiiiiii Success !!!! "<<endl;
                    current_label->c1 = get_worst_veh_cost(trucks_tours);
                    current_label->c2 = get_worst_drone_cost(drone_assignment);
                    cout<<"After Moves current_label = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;
                }
            }
*/


            //update drone_list
            drone_list.clear();
            for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
                for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                    drone_list.push_back(*it3);
                }
            }


            cout<<endl;
            cout<<"********* Trucks_tours Apres MOVES *********"<<endl;
            //int taille = trucks_tours.size();
            for(int i=0; i<NB_VEH; i++){
                cout<<endl;
                cout<<"** Tour du vehicule "<<i<<" **"<<endl;
                for(it4 = trucks_tours[i].begin(); it4!=trucks_tours[i].end(); ++it4)
                    cout<<*it4<<";";
                cout<<endl;
            }
            cout<<"***** liste drones Apres MOVES *****"<<endl;
            for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
                for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                    cout<< *it3<<",";
                }
                cout<<endl;
            }


            /******* Step 4 : concatenate vehicle's tours randomly *******/

            truck_tour.clear();
            values.clear();
            for (int i=0;i<NB_VEH;i++)
            {
                do
                {
                    nb=rand()%NB_VEH;
                    //check or number is already used:
                    check=true;
                    for (int j=0;j<i;j++)
                        if (nb == values[j]) //if number is already used
                        {
                            check=false; //set check to false
                            break; //no need to check the other elements of value[]
                        }
                } while (!check); //loop until new, unique number is found
                values.push_back(nb); //store the generated number in the array
            }

            //Verification
            cout<<"*** Ordre de concatenation ***"<<endl;
            for(int i=0; i< values.size(); i++)
                cout<<values[i]<<";";
            cout<<endl;

            //concatenation
            copy_deque_to_deque(trucks_tours[values[0]],truck_tour);
            for(int i=1; i<values.size(); i++){
                concate(truck_tour,trucks_tours[values[i]]);
            }

            //Verification
            /*cout<<"***** Tour vehicule apres concatenation *****"<<endl;
            for(it3 = truck_tour.begin(); it3!=truck_tour.end(); ++it3){
                cout<<*it3<<";";
            }
            cout<<endl;
            cout<<"***** liste drones Apres MOVES*****"<<endl;
            for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
                for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                    cout<< *it3<<",";
                }
                cout<<endl;
            }*/


            /******* Step 5 : best insertion (to have a new giant_tsp) *******/

            for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
                for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                    best_insertion(truck_tour, *it3);
                }
            }


            //we delete the old tour
            for(it = tour.begin(); it!=tour.end(); ++it)
                delete *it;
            tour.clear();

            //we build the new giant tour
            for(it3 = truck_tour.begin(); it3!=truck_tour.end(); ++it3){
                tour.push_back(new Node(*find_node_by_id(*it3)));
            }

            //Verification
            /*cout<<endl;
            cout<<"***** NOUVEAU TOUR CONSTRUIT *****"<<endl;
            for(it = tour.begin(); it!=tour.end(); ++it){
                cout<<(*it)->get_id()<<";";
            }
            cout<<endl;*/
            //cout<<"COUT = "<<get_truck_cost(truck_tour)<<endl;
            //cout<<endl;


            if(current_label->c2 > 0){
                averageGapDrone = averageGapDrone + fabs(100.*((droneCostFromSplit-current_label->c2)/current_label->c2));
                if(fabs(100.*((droneCostFromSplit-cost)/cost))> maxGapDrone)
                   maxGapDrone = fabs(100.*((droneCostFromSplit-current_label->c2)/current_label->c2));
            }

            cout<<endl;
            cout<<"current_label = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;
            cout<<"recor_label = ("<<recor_label->c1<<","<<recor_label->c2<<")"<<endl;
            cout<<endl;

            if(compare_solutions1(current_label,recor_label, trucks_tours,drone_assignment,best_trucks_tours, best_drone_assignment)){//True if current_label is better than recor_label
                /*cout<<"Test Affichage tours Veh trucks_tours"<<endl;
                for(it2 = trucks_tours.begin(); it2!=trucks_tours.end(); ++it2){
                    for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3)
                        cout<< *it3<<",";
                    cout<<endl;
                }*/

                recor_label->c1 = current_label->c1;
                recor_label->c2 = current_label->c2;

                copy_dequetab_to_dequetab(trucks_tours,best_trucks_tours);
                copy_deque_to_deque(drone_list,best_drone_list);
                copy_dequetab_to_dequetab(drone_assignment, best_drone_assignment);

                cmpt = 0;

            }else{//the current solution is not better than the recor solution
                //cout<<"STOP!!!"<<endl;
                improve = false;
                cmpt++;
                //we delete tour to free the memory
                /*for(it = tour.begin(); it!=tour.end(); ++it)
                    delete *it;
                tour.clear();*/
            }

            for(it2 = trucks_tours.begin(); it2!=trucks_tours.end(); ++it2)
                (*it2).clear();
            trucks_tours.clear();
            drone_list.clear();
            for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2)
                (*it2).clear();
            drone_assignment.clear();

            nb_iteration++;
            //cout<<"On arrive ici"<<endl;

        }

        //we delete the tour
        std::for_each(tour.begin(), tour.end(), Delete());
        tour.clear();


        if(nb_restart == 0){

            final_recor_label->c1 = recor_label->c1;
            final_recor_label->c2 = recor_label->c2;

            copy_dequetab_to_dequetab(best_trucks_tours, final_trucks_tours);
            copy_deque_to_deque(best_drone_list, final_drone_list);
            copy_dequetab_to_dequetab(best_drone_assignment, final_drone_assignment);

        }else{
            //cout<<"j'entre ici"<<endl;
            if(compare_solutions1(recor_label,final_recor_label,best_trucks_tours, best_drone_assignment, final_trucks_tours, final_drone_assignment)){//final_recor_label is dominated by recor_label
                final_recor_label->c1 = recor_label->c1;
                final_recor_label->c2 = recor_label->c2;

                /*cout<<"Test Affichage tours Veh best_trucks_tours"<<endl;
                for(it2 = best_trucks_tours.begin(); it2!=best_trucks_tours.end(); ++it2){
                    for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3)
                        cout<< *it3<<",";
                    cout<<endl;
                }*/

                copy_dequetab_to_dequetab(best_trucks_tours, final_trucks_tours);
                copy_deque_to_deque(best_drone_list, final_drone_list);
                copy_dequetab_to_dequetab(best_drone_assignment, final_drone_assignment);
            }
        }

        for(it2 = best_trucks_tours.begin(); it2!=best_trucks_tours.end(); ++it2)
            (*it2).clear();
        best_trucks_tours.clear();
        best_drone_list.clear();
        for(it2 = best_drone_assignment.begin(); it2!=best_drone_assignment.end(); ++it2)
            (*it2).clear();
        best_drone_assignment.clear();

        //cout<<"final recor est ("<<final_recor_label->c1<<","<<final_recor_label->c2<<")"<<endl;

        //averageNbGenLab = averageNbGenLab/(nb_iteration-1);
        //averageNbDelLab = averageNbDelLab/(nb_iteration-1);
        //averageGapDrone = averageGapDrone/(nb_iteration-1);

        //averageGlobalNbGenLab = averageGlobalNbGenLab + averageNbGenLab;
        //averageGlobalNbDelLab = averageGlobalNbDelLab + averageNbDelLab;
        //averageGlobalGapDrone = averageGlobalGapDrone + averageGapDrone;
        //system("pause");
        nb_restart++;
        //if(droneEligible.empty())
        //delete giant_tsp;

    }

    if(averageNbGenLab != 0)
        percentDelLab = ((double)averageNbDelLab/ (double)averageNbGenLab)*100.;

    if(nbSplitCall != 0){

    //cout<<"averageGapDrone = "<<averageGapDrone<<endl;

    averageNbGenLab = averageNbGenLab/nbSplitCall;
    //averageNbDelLab = averageNbDelLab/nbSplitCall;
    averageGapDrone = averageGapDrone/nbSplitCall;
    //averageGlobalNbGenLab = averageGlobalNbGenLab/(nb_restart);
    //averageGlobalNbDelLab = averageGlobalNbDelLab/(nb_restart);
    //averageGlobalGapDrone = averageGlobalGapDrone/(nb_restart);
    nbSplitCall = nbSplitCall/(nb_restart);
    }

    /*cout<<"liste des clients servis par le drone"<<endl;
    for(it3 = final_drone_list.begin(); it3!=final_drone_list.end(); ++it3){
        cout<<*it3<<endl;
    }*/

    //}
    //cout<<"maxGapDrone = "<<maxGapDrone<<endl;
    //cout<<"avgGapDrone = "<<averageGapDrone<<endl;

    int nbCliDrone = final_drone_list.size();
    int nbCliVeh = 0;


    cout<<"final-recor_label = ("<<final_recor_label->c1<<","<<final_recor_label->c2<<")"<<endl;
    //cout<<endl;
    //cout<<"Final trucks tours size = "<<final_trucks_tours.size()<<endl;

    for(it2 = final_trucks_tours.begin(); it2!=final_trucks_tours.end(); ++it2){
        /*for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3)
            cout<< *it3<<",";
        cout<<endl;*/
        nbCliVeh += ((*it2).size()-2);
    }
    nbCliDrone = final_drone_list.size();


    //We write the solution in the result file
    write_result(filename,file_n.c_str(),final_recor_label, nbCliVeh, nbCliDrone, (double)(clock() - tStart)/CLOCKS_PER_SEC,timeSplit,timeOptimize,timeLocalSearch, nb_restart, nbSplitCall, averageNbGenLab, percentDelLab, averageGapDrone);

    delete current_label;
    delete recor_label;
    delete final_recor_label;
}

//Run the ILS better walk metaHeuristic (case K vehicles)
void Heuristics::run_ILS(string path, string file_n)
{
    clock_t tStart = clock(); //to have the execution time at the end
    Label2 *curSol;
    Label * current_label;
    int nb_iteration, nbSplitCall = 0, averageNbGenLab=0, averageNbDelLab=0, nb_restart = 0, nbCliVeh, nbCliDrone;
    int worstTourPos, posInsert, nb, destDepotId = all_customers.size(), i, j, X;
    double cost, averageGapDrone = 0., maxGapDrone = 0., droneCostFromSplit, percentDelLab=0., worstTourCost, worstDrone, makespan, timeSplit=0, timeOptimize=0, timeLocalSearch=0;
    deque<Node*>::iterator it;  //An iterator on Node list
    deque<deque <int> >::iterator it2;
    deque<int>::iterator it3, it4;  //An iterator on int list
    deque<int> listeN, liste;
    deque<int> values;
    double tourCost[NB_VEH]; //cost of each vehicle tour
    bool tourOpt[NB_VEH]; //indicates if a tour has already been optimized
    tsp *giant_tsp;
    ostringstream s1, s2, s3,s4;
    s1 << NB_VEH;
    s2 << NB_DRONE;
    s3 << SPEED;
    s4 << LIMIT_LAB_METHOD;
    string path1 = path+"/result_"+s1.str()+"_"+s2.str()+"_"+s3.str()+"_"+s4.str()+".csv", path2,str;
    const char *filename = path1.c_str();
    bool improve, optimize, check, success=false;

    current_label = new Label();
    recor_label = new Label();

    cout<<endl;
    cout<<"___________________ LES DONNEES DU PROPBLEME ____________________"<<endl;
    cout<<"=================================================================="<<endl;
    cout<<"Nombre de clients = "<<all_customers.size()-1<<endl;
    cout<<"Nombre de vehicules = "<<NB_VEH<<endl;
    cout<<"Nombre de drones = "<<NB_DRONE<<endl;
    cout<<"Temps d'execution limite = "<<TIME_LIMIT<<" s"<<endl;
    cout<<"Nombre de clients drone Eligible = "<<droneEligible.size()<<endl;
    cout<<"****** Liste des clients drone Eligibles ******"<<endl;
    for(it3 = droneEligible.begin(); it3!=droneEligible.end(); ++it3)
        cout<<*it3<<";";
    cout<<endl;
    cout<<"=================================================================="<<endl;
    cout<<endl;


    cout<<endl;
    cout<<"________________________ PROCEDURE ILS ____________________________"<<endl;

    /******* Initialization : Build a giant TSP *******/

    //tsp with 2-opt
    giant_tsp = new tsp(all_customers); //creates a tsp object
    cout<<endl;
    cout<<"################ Initialization : Build a giant TSP ################"<<endl;
    cout<<endl;

    cost = giant_tsp->run_tsp(time(NULL) + nb_restart); //the result is found in solution deque

    cout<<"****** Voici le tour geant construit ******"<<endl;
    for(it3=giant_tsp->solution.begin(); it3!=giant_tsp->solution.end(); ++it3){
        tour.push_back(new Node(*find_node_by_id(*it3)));
        cout<<(*it3)<<"->";
    }
    cout<<"0"<<endl;
    cout<<endl;
    cout <<"Cout du tour TSP = "<<cost<<endl;




    //we add the depot at the end of tour
    tour[0]->set_drone(1); //the origin depot node is imposed to the truck
    tour.push_back(new Node(*tour[0]));

    int taille = tour.size();
    tour[taille-1]->set_id(destDepotId);

    //cout<<"Taille du tour geant = "<<tour.size()<<endl;

    //we set tour as the best_trucks_tours found so far
    best_trucks_tours.clear();
    liste.clear();

    for(it = tour.begin(); it!=tour.end(); ++it)
        liste.push_back((*it)->get_id());
    best_trucks_tours.push_back(liste);

    recor_label->c1 = get_truck_cost(liste);
    recor_label->c2 = 0;
    //cout<<"recor_label = ("<<recor_label->c1<<","<<recor_label->c2<<")"<<endl;

    using namespace std::chrono;

    X = NB_VEH;
    while((double)(clock() - tStart)/CLOCKS_PER_SEC <= TIME_LIMIT){

        nb_iteration = 1;
        improve = true;

        /******* Decoding : Split procedure on tour  *******/

        // Get starting timepoint
        auto start = high_resolution_clock::now();

        curSol = split2(tour,nb_iteration,LIMIT_LAB_METHOD);
        // Get ending timepoint
        auto stop = high_resolution_clock::now();

        std::chrono::duration<double> diff = stop-start;
        timeSplit = timeSplit + diff.count();

        std::cout << "Temps d'execution Split "<< diff.count() << " s\n";
        cout<<endl;

        nbSplitCall ++; //counts the number of split calls
        averageNbGenLab = averageNbGenLab + nbGenLab;
        averageNbDelLab = averageNbDelLab + nbDelLab;

        current_label->c1 = max(curSol->c_gloVeh,curSol->c1) ;
        current_label->c2 = curSol->c2;
        droneCostFromSplit = curSol->c2;

        /******* Route reoptimization + Drone assignment (LKH + PMS) *******/

        cout<<endl;
        cout<<"################ Route reoptimization + Drone assignment (LKH + PMS) ################"<<endl;
        cout<<endl;


        start = high_resolution_clock::now();

        int len = trucks_tours.size();
        for(int i=0; i<len; i++)
            tourCost[i] = get_truck_cost(trucks_tours[i]);

        for(int i=0; i<len; i++){
            if(trucks_tours[i].size() > 3){
                trucks_tours[i].pop_back();
                create_inputFile_forLkh(trucks_tours[i]);
                cost = run_lkh(trucks_tours[i]);
                trucks_tours[i].clear();
                copy_deque_to_deque(truck_tour, trucks_tours[i]);
                tourCost[i] = cost;
            }
        }

        //we search the worst vehicle tour
        worstTourPos = 0;
        worstTourCost = tourCost[0];
        for(int i=1; i<len; i++){
            if(tourCost[i]>worstTourCost){
                worstTourCost = tourCost[i];
                worstTourPos = i;
            }
        }

        //we set the current_label
        current_label->c1 = worstTourCost;
        cost = lpt_heuristic();
        current_label->c2 = cost;


        //VERIFICATION
        cout<<"------ Solution apres OPTIMISATION ------"<<endl;
        //int taille = trucks_tours.size();
        cout<<endl;
        cout<<"****** VEHICULES ******"<<endl;
        for(int i=0; i<NB_VEH; i++){
            cout<<endl;
            cout<<"Tour du vehicule "<<i+1<<": ";
            for(it4 = trucks_tours[i].begin(); it4!=trucks_tours[i].end()-1; ++it4)
                cout<<*it4<<"->";
            cout<<"0";
            cout<<endl;
            cout<<"Cout vehicule "<<i+1<<" = "<<get_truck_cost(trucks_tours[i]);
            cout<<endl;
        }

        i = 1;
        cout<<endl;
        cout<<"****** DRONES ******"<<endl;
        for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
            cout<<endl;
            cout<<"Liste clients drone "<<i<<" : ";
            for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                cout<< *it3<<";";
            }
            cout<<endl;
            cout<<"Cout drone "<<i<<" = "<<get_drone_cost(*it2)<<endl;
            i++;
        }

       cout<<endl;
       cout<<"currentSol apres LKH + PMS = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;

        /*cout<<"********* Trucks_tours Apres OPTIMISATION *********"<<endl;
        //int taille = trucks_tours.size();
        for(int i=0; i<NB_VEH; i++){
            cout<<endl;
            cout<<"** Tour du vehicule "<<i<<" **"<<endl;
            for(it4 = trucks_tours[i].begin(); it4!=trucks_tours[i].end(); ++it4)
                cout<<*it4<<";";
            cout<<endl;
        }
        cout<<"***** liste drones Apres OPTIMISATION *****"<<endl;
        for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
            for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                cout<< *it3<<",";
            }
            cout<<endl;
        }*/
        stop = high_resolution_clock::now();
        diff = stop-start;
        timeOptimize = timeOptimize + diff.count();

        std::cout << "Temps d'execution Optimize "<< diff.count() << " s\n";
        cout<<endl;


        start = high_resolution_clock::now();

        /******** Local search : moves operations *********/
        cout<<endl;
        cout<<"################ Local search : moves operations ################"<<endl;
        cout<<endl;


        improve = true;
        while(improve){
            sortCost(trucks_tours,drone_assignment);
            worstTourCost = get_truck_cost(trucks_tours[0]);
            worstDrone = get_drone_cost(drone_assignment[0]);
            makespan = max(worstTourCost, worstDrone);
            cout<<"--------------------------------------------------------------"<<endl;
            cout<<endl;

            cout<<"Makespan courant = "<<makespan<<endl;

            if(worstTourCost > worstDrone){ //makespan is given by a vehicle
                cout<<"Le makespan est induit par un vehicule"<<endl;
                success = false;
                j=drone_assignment.size()-1;
                cout<<endl;
                cout<<"--------------- Tests TRANSFER MOVE Veh -> Dro ---------------"<<endl;
                cout<<endl;

               /* cout<<"Parametre 1 - Vehicule : ";
                for(it4 = trucks_tours[0].begin(); it4!=trucks_tours[0].end()-1; ++it4)
                    cout<<*it4<<"->";
                cout<<"0";
                cout<<"    Cout = "<<get_truck_cost(trucks_tours[0]);
                cout<<endl;
                */

                while(j>=0 && !success){
                    //cout<<" &&&&&&&&& drone j = "<<j<<" &&&&&&&&&&&&&&&"<<endl;
                    success = transferMove1(trucks_tours[0],drone_assignment[j],makespan);
                    j--;
                }

                if(!success){
                    cout<<endl;
                    cout<<"====== !!!!!! Echec de tous les TRANSFER MOVE Veh -> Dro !!!!!! ======" <<endl;

                    cout<<endl;
                    cout<<"--------------- Tests EXCHANGE MOVE Veh-Dro ---------------"<<endl;
                    cout<<endl;

                    /*cout<<"Parametre 1 - Vehicule : ";
                    for(it4 = trucks_tours[0].begin(); it4!=trucks_tours[0].end()-1; ++it4)
                        cout<<*it4<<"->";
                    cout<<"0";
                    cout<<"    Cout = "<<get_truck_cost(trucks_tours[0]);
                    cout<<endl;*/


                    j=drone_assignment.size()-1;
                    while(j>=0 && !success){
                        success = exchangeMove2(trucks_tours[0],drone_assignment[j],makespan);
                        j--;
                    }
                    if(!success){
                        cout<<endl;
                        cout<<"====== !!!!!! Echec de tous les EXCHANGE MOVE Veh-Dro !!!!!! ======"<<endl;

                        cout<<endl;
                        cout<<"--------------- Tests RELOCATE MOVE Veh-Veh ---------------"<<endl;
                        cout<<endl;

                        /*cout<<"Parametre 1 - Vehicule : ";
                        for(it4 = trucks_tours[0].begin(); it4!=trucks_tours[0].end()-1; ++it4)
                            cout<<*it4<<"->";
                        cout<<"0";
                        cout<<"    Cout = "<<get_truck_cost(trucks_tours[0]);
                        cout<<endl;*/

                        i=trucks_tours.size()-1;
                        while(i>1 && !success){
                            //cout<<" &&&&&&&&& vehicule i = "<<i<<" &&&&&&&&&&&&&&&"<<endl;
                            success = relocateMove(trucks_tours[0],trucks_tours[i],makespan);
                            i--;
                        }

                        if(!success){
                            cout<<endl;
                            cout<<"====== !!!!!! Echec de tous les RELOCATE MOVE !!!!!! ======"<<endl;

                            cout<<endl;
                            cout<<"--------------- Tests EXCHANGE MOVE Veh-Veh ---------------"<<endl;
                            cout<<endl;

                            /*cout<<"Parametre 1 - Vehicule : ";
                            for(it4 = trucks_tours[0].begin(); it4!=trucks_tours[0].end()-1; ++it4)
                                cout<<*it4<<"->";
                            cout<<"0";
                            cout<<"    Cout = "<<get_truck_cost(trucks_tours[0]);
                            cout<<endl;*/


                            i=trucks_tours.size()-1;
                            while(i>1 && !success){
                                success = exchangeMove1(trucks_tours[0],trucks_tours[i],makespan);
                                i--;
                            }
                            if(!success){
                                cout<<"====== !!!!!! Echec de tous les EXCHANGE MOVE Veh-Veh !!!!!! ======"<<endl;
                                i=trucks_tours.size()-1;
                                while(i>1 && !success){
                                    success = crossMove(trucks_tours[0],trucks_tours[i],makespan);
                                    i--;
                                }
                                if(!success){
                                    cout<<"====== !!!!!! Echec de tous les CROSS MOVE Veh-Veh !!!!!! ======"<<endl;
                                    improve = false;
                                }
                            }
                        }
                    }
                }
                if(success){
                    //cout<<"Wouppiiiiiiiiiiiii Success !!!! On continu"<<endl;
                    current_label->c1 = get_worst_veh_cost(trucks_tours);
                    current_label->c2 = get_worst_drone_cost(drone_assignment);
                    cout<<"Nouvelle Solution Courante = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;
                    cout<<endl;
                }

            }else{//makespan is given by a drone
                cout<<"Le makespan est induit par un drone"<<endl;
                //cout<<endl;
                /*cout<<"Parametre 1 - Drone : ";
                for(it4 = drone_assignment[0].begin(); it4!=drone_assignment[0].end(); ++it4)
                    cout<< *it4<<";";
                cout<<"   Cout = "<<get_drone_cost(drone_assignment[0])<<endl;*/

                success = false;
                i=trucks_tours.size()-1;
                while(i>=0 && !success){
                    success = transferMove2(drone_assignment[0], trucks_tours[i], makespan);
                    i--;
                }
                if(!success){
                    cout<<"====== !!!!!! Echec de tous les TRANSFER MOVE Dro -> Veh !!!!!! ======"<<endl;
                    i=trucks_tours.size()-1;
                    while(i>=0 && !success){
                        success = exchangeMove2(trucks_tours[i], drone_assignment[0], makespan);
                        i--;
                    }
                    if(!success){
                        cout<<"====== !!!!!! Echec de tous les EXCHANGE MOVE Veh-Dro !!!!!! ======"<<endl;
                        improve = false;
                    }
                }
                if(success){
                    //cout<<"Wouppiiiiiiiiiiiii Success !!!! On continu "<<endl;
                    current_label->c1 = get_worst_veh_cost(trucks_tours);
                    current_label->c2 = get_worst_drone_cost(drone_assignment);
                    cout<<"Nouvelle Solution Courante = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;
                    cout<<endl;
                }
            }
        }
        stop = high_resolution_clock::now();

        //VERIFICATION
        cout<<endl;
        cout<<"------ Solution apres RECHERCHE LOCALE ------"<<endl;
        //int taille = trucks_tours.size();
        cout<<endl;
        cout<<"****** VEHICULES ******"<<endl;
        for(int i=0; i<NB_VEH; i++){
            cout<<endl;
            cout<<"Tour du vehicule "<<i+1<<": ";
            for(it4 = trucks_tours[i].begin(); it4!=trucks_tours[i].end()-1; ++it4)
                cout<<*it4<<"->";
            cout<<"0";
            cout<<endl;
            cout<<"Cout vehicule "<<i+1<<" = "<<get_truck_cost(trucks_tours[i]);
            cout<<endl;
        }

        i = 1;
        cout<<endl;
        cout<<"****** DRONES ******"<<endl;
        for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
            cout<<endl;
            cout<<"Liste clients drone "<<i<<" : ";
            for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                cout<< *it3<<";";
            }
            cout<<endl;
            cout<<"Cout drone "<<i<<" = "<<get_drone_cost(*it2)<<endl;
            i++;
        }

        diff = stop-start;
        timeLocalSearch = timeLocalSearch + diff.count();

        cout<<endl;
        std::cout << "Temps d'execution Local search "<< diff.count() << " s\n";

        //update drone_list
        drone_list.clear();
        for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
            for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                drone_list.push_back(*it3);
            }
        }

        if(current_label->c2 > 0){
            averageGapDrone = averageGapDrone + fabs(100.*((droneCostFromSplit-current_label->c2)/current_label->c2));
            if(fabs(100.*((droneCostFromSplit-cost)/cost))> maxGapDrone)
               maxGapDrone = fabs(100.*((droneCostFromSplit-current_label->c2)/current_label->c2));
        }



        /******* Update current solution : comparing currentSol en bestSol *******/

        cout<<endl;
        cout<<"################ Update bestSol ################"<<endl;
        cout<<endl;


        //True if current_label is better than recor_label
        if(compare_solutions1(current_label,recor_label, trucks_tours,drone_assignment,best_trucks_tours, best_drone_assignment)){

            recor_label->c1 = current_label->c1;
            recor_label->c2 = current_label->c2;

            copy_dequetab_to_dequetab(trucks_tours,best_trucks_tours);
            copy_deque_to_deque(drone_list,best_drone_list);
            copy_dequetab_to_dequetab(drone_assignment, best_drone_assignment);

            X = NB_VEH;
        }else{//the current solution is not better than the recor solution
            X = max(0, X-1);
        }



        cout<<endl;
        //cout<<"current_label = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;
        //cout<<"recor_label = ("<<recor_label->c1<<","<<recor_label->c2<<")"<<endl;
        //cout<<"currentSol = ("<<current_label->c1<<","<<current_label->c2<<")"<<endl;
        cout<<"bestSol = ("<<recor_label->c1<<","<<recor_label->c2<<")"<<endl;

        cout<<endl;


        /******* New giant tour construction *******/
        cout<<endl;
        cout<<"################ New giant tour construction (Dominique method) ################"<<endl;
        cout<<endl;

        //new_giant_tour_construction_laurent(truck_tour,best_trucks_tours,best_drone_assignment);
        new_giant_tour_construction_dominique(truck_tour,best_trucks_tours,best_drone_assignment, X);


        cout<<endl;
        cout<<"****** Nouveau tour geant ******"<<endl;
        for(it3 = truck_tour.begin(); it3!=truck_tour.end(); ++it3){
            cout<<*it3<<";";
        }
        cout<<endl;


        /******* Concatenate vehicle's tours randomly *******/

        /*truck_tour.clear();
        values.clear();
        for (int i=0;i<NB_VEH;i++)
        {
            do
            {
                nb=rand()%NB_VEH;
                //check or number is already used:
                check=true;
                for (int j=0;j<i;j++)
                    if (nb == values[j]) //if number is already used
                    {
                        check=false; //set check to false
                        break; //no need to check the other elements of value[]
                    }
            } while (!check); //loop until new, unique number is found
            values.push_back(nb); //store the generated number in the array
        }

        //Verification
        cout<<"*** Ordre de concatenation ***"<<endl;
        for(int i=0; i< values.size(); i++)
            cout<<values[i]<<";";
        cout<<endl;

        //concatenation
        copy_deque_to_deque(best_trucks_tours[values[0]],truck_tour);
        for(int i=1; i<values.size(); i++){
            concate(truck_tour,best_trucks_tours[values[i]]);
        }

        //Verification
        /*cout<<"***** Tour vehicule apres concatenation *****"<<endl;
        for(it3 = truck_tour.begin(); it3!=truck_tour.end(); ++it3){
            cout<<*it3<<";";
        }
        cout<<endl;
        cout<<"***** liste drones Apres MOVES*****"<<endl;
        for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
            for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                cout<< *it3<<",";
            }
            cout<<endl;
        }*/


        /******* Best insertion *******/

        /*for(it2 = best_drone_assignment.begin(); it2!=best_drone_assignment.end(); ++it2){
            for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                best_insertion(truck_tour, *it3);
            }
        }*/

        /********* Random Insertion *******/
        /*for(it2 = best_drone_assignment.begin(); it2!=best_drone_assignment.end(); ++it2){
            for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                random_insertion(truck_tour, *it3);
            }
        }
        cout<<"taille truck_tour après random insertion = "<<truck_tour.size()<<endl;
        cout<<"cout truck_tour avant 2-opt = "<<get_truck_cost(truck_tour)<<endl;

        /****** 2-opt ******/

        //truck_tour.pop_back();
        //amelioration();

        //cout<<"taille truck_tour après 2-opt = "<<truck_tour.size()<<endl;
        //cout<<"cout truck_tour apres 2-opt = "<<get_truck_cost(truck_tour)<<endl;

        //we delete the old tour
        for(it = tour.begin(); it!=tour.end(); ++it)
            delete *it;
        tour.clear();

        //we build the new giant tour
        for(it3 = truck_tour.begin(); it3!=truck_tour.end(); ++it3){
            tour.push_back(new Node(*find_node_by_id(*it3)));
        }

        //Verification
        /*cout<<endl;
        cout<<"***** NOUVEAU TOUR CONSTRUIT *****"<<endl;
        for(it = tour.begin(); it!=tour.end(); ++it){
            cout<<(*it)->get_id()<<";";
        }
        cout<<endl;*/


        for(it2 = trucks_tours.begin(); it2!=trucks_tours.end(); ++it2)
            (*it2).clear();
        trucks_tours.clear();
        drone_list.clear();
        for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2)
            (*it2).clear();
        drone_assignment.clear();

        nb_iteration++;

    }

    if(averageNbGenLab != 0)
        percentDelLab = ((double)averageNbDelLab/ (double)averageNbGenLab)*100.;

    if(nbSplitCall != 0){
        averageNbGenLab = averageNbGenLab/nbSplitCall;
        averageGapDrone = averageGapDrone/nbSplitCall;
        //averageGlobalNbGenLab = averageGlobalNbGenLab/(nb_restart);
        //averageGlobalNbDelLab = averageGlobalNbDelLab/(nb_restart);
        //averageGlobalGapDrone = averageGlobalGapDrone/(nb_restart);
    }


    //cout<<"final-recor_label = ("<<recor_label->c1<<","<<recor_label->c2<<")"<<endl;
    cout<<endl;
    cout<<"_______________________ OUTPUT DU PROBLEME _______________________"<<endl;
    cout<<"=================================================================="<<endl;
    //cout<<"Nombre de vehicules = "<<NB_VEH<<endl;
    //cout<<"Nombre de drones = "<<NB_DRONE<<endl;

    cout<<"bestSol Finale = ("<<recor_label->c1<<","<<recor_label->c2<<")"<<endl;
    cout<<"Nombre D'appel du SPLIT = "<<nbSplitCall<<endl;
    cout<<"Temps d'execution total reel = "<<(double)(clock() - tStart)/CLOCKS_PER_SEC<<" s"<<endl;
    cout<<"Temps D'execution total SPLIT = "<<timeSplit<<" s"<<endl;
    cout<<"Temps D'execution total OMPTIMIZE = "<<timeOptimize<<" s"<<endl;
    cout<<"Temps D'execution total LOCAL SERACH = "<<timeLocalSearch<<" s"<<endl;

        //VERIFICATION
        cout<<endl;
        cout<<"------ Solution FINALE ------"<<endl;
        //int taille = trucks_tours.size();
        cout<<endl;
        cout<<"****** VEHICULES ******"<<endl;
        for(int i=0; i<NB_VEH; i++){
            cout<<endl;
            cout<<"Tour du vehicule "<<i+1<<": ";
            for(it4 = best_trucks_tours[i].begin(); it4!=best_trucks_tours[i].end()-1; ++it4)
                cout<<*it4<<"->";
            cout<<"0";
            cout<<endl;
            cout<<"Cout vehicule "<<i+1<<" = "<<get_truck_cost(best_trucks_tours[i]);
            cout<<endl;
        }

        i = 1;
        cout<<endl;
        cout<<"****** DRONES ******"<<endl;
        for(it2 = best_drone_assignment.begin(); it2!=best_drone_assignment.end(); ++it2){
            cout<<endl;
            cout<<"Liste clients drone "<<i<<" : ";
            for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                cout<< *it3<<";";
            }
            cout<<endl;
            cout<<"Cout drone "<<i<<" = "<<get_drone_cost(*it2)<<endl;
            i++;
        }

    cout<<"=================================================================="<<endl;
    cout<<endl;




    nbCliVeh = 0;
    for(it2 = best_trucks_tours.begin(); it2!=best_trucks_tours.end(); ++it2){
        nbCliVeh += ((*it2).size()-2);
    }
    nbCliDrone = best_drone_list.size();
    nb_restart = 0;

    //We write the solution in the result file
    //write_result(filename,file_n.c_str(),recor_label, nbCliVeh, nbCliDrone, (double)(clock() - tStart)/CLOCKS_PER_SEC, nb_restart, nbSplitCall, averageNbGenLab, percentDelLab, averageGapDrone);
    write_result_ILS2(filename,file_n.c_str(),recor_label, nbCliVeh, nbCliDrone, (double)(clock() - tStart)/CLOCKS_PER_SEC, timeSplit, timeOptimize, timeLocalSearch, nbSplitCall, averageNbGenLab, percentDelLab, averageGapDrone);

    delete current_label;
    delete recor_label;
    delete final_recor_label;
}










































//Apply inversion operation
 void Heuristics::apply_inversion(int i, int j)
{
    int index1, index2;

    if(j>i){
       index1 = i;
       index2 = j;
    }else{
       index1 = j;
       index2 = i;
    }

    while(index2 > index1){
        apply_exchange(index1,index2);
        index1 ++;
        index2 --;
    }
}

//Apply inversion operation
void Heuristics::apply_exchange(int i, int j)
{
    int c;

    c = truck_tour[j];
    truck_tour[j] = truck_tour[i];
    truck_tour[i] = c;
}

//Returns true if applying 2 opt move will yield a cost gain
 bool Heuristics::gain_cost(int i, int j){

    double cost_before = 0; //sum of the deleted edges cost
    double cost_after = 0; //sum of the added edges cost
    int index1, index2, i1, j1;

    int taille = truck_tour.size();
    if(j>i){
       index1 = i;
       index2 = j;
    }else{
       index1 = j;
       index2 = i;
    }

    i1 = index1-1;
    j1 = index2+1;
    if(i1 == -1)
        i1 = taille-1;
    if(j1 == taille)
        j1 = 0;


    cost_before = matrixTruck[truck_tour[i1]][truck_tour[index1]] +
                  matrixTruck[truck_tour[index2]][truck_tour[j1]];

    cost_after = matrixTruck[truck_tour[i1]][truck_tour[index2]] +
                 matrixTruck[truck_tour[index1]][truck_tour[j1]];

    if(cost_after < cost_before){
        return true;
    }else
        return false;
}

//Apply amelioration with 2-opt moves
 double Heuristics::amelioration(){

    int taille = truck_tour.size();
    bool modif = true;

    while(modif)
    {
        modif = false;

        for(int i=1; i<taille-1; i++){//test all possible 2-opt moves
            for(int j=i+1; j<taille; j++){
                if (gain_cost(i,j)){
                    apply_inversion(i,j);
                    modif = true;
                }
            }
        }
    }
    truck_tour.push_back(0);
    return get_truck_cost(truck_tour);
 }

//Creates the input file for LKH
void Heuristics::create_inputFile_forLkh(deque <int> listeN){

    string const nomFichier("Instances/For_LKH/route.tsp");
    ofstream monFlux(nomFichier.c_str());
    deque<int>::iterator it1, it2;
    if(monFlux)
    {
        monFlux << "NAME: tour" << endl;
        monFlux << "TYPE: TSP" << endl;
        monFlux << "COMMENT: tour" << endl;
        monFlux << "DIMENSION: "<<listeN.size() << endl;
        monFlux << "EDGE_WEIGHT_TYPE: EXPLICIT"<< endl;
        monFlux << "EDGE_WEIGHT_FORMAT: FULL_MATRIX"<< endl;
        monFlux << "EDGE_WEIGHT_SECTION"<< endl;

         for(it1 = listeN.begin(); it1!=listeN.end(); ++it1){
                for(it2 = listeN.begin(); it2!=listeN.end(); ++it2){
                    monFlux << (int) round(matrixTruck[(*it1)][(*it2)]) <<"\t";
                }
                monFlux<<endl;
         }
         monFlux << "EOF";
    }
    else
    {
        cout << "ERREUR: Impossible d'ouvrir le fichier." << endl;
    }
}

double Heuristics::run_lkh(deque <int> listeN){

    system("LKH\\LKH.exe LKH\\Parameter.Win > tempfile.tmp");

    string mon_fichier = "Instances/For_LKH/solution.sol";
    ifstream fichier(mon_fichier.c_str(), ios::in);
    int i=0, n=5, j;
    deque<int>::iterator it;
    deque<int> solution;
    double cost=0;

    solution.clear();
    if(fichier)
    {
        string ligne;
        while(getline(fichier, ligne))
        {
                if(ligne.compare("EOF")!=0 && i>= n){
                    if(atoi(ligne.c_str()) != -1)
                        solution.push_back(listeN[atoi(ligne.c_str())-1]);
                }
                i++;
        }

        //verification
        /*cout<<"voici le tour"<<endl;
        for(it = solution.begin(); it!=solution.end(); ++it){
            cout<<(*it)<<";";
        }
        cout<<endl;*/

        //permute tour such that depot is at the beginning

        while(solution[0] != 0){
            j = solution[0];
            solution.pop_front();
            solution.push_back(j);
        }

        //verification
        truck_tour.clear();
        //cout<<"voici le tour apres permutation"<<endl;
        for(it = solution.begin(); it!=solution.end(); ++it){
            truck_tour.push_back(*it);
            //cout<<"it dans runLKH = "<<(*it)<<";";
        }
        //cout<<endl;

        truck_tour.push_back(0);

        //we calculate real cost of the solution
        cost = get_truck_cost(truck_tour);
        fichier.close();
    }
     else
        cout << "Erreur à l'ouverture !" << endl;

     return cost;
}



 //Returns a Node given his id by searching in all_customers list
Node* Heuristics::find_node_by_id(int id){
    deque<Node*>::iterator it;  //An iterator on Node list
    Node* n = NULL;

    for(it = all_customers.begin(); it!=all_customers.end(); ++it){
        if((*it)->get_id() == id){
            n = *it;
            break;
        }
    }
    return n;
}

//Returns true if id is found in tour
bool Heuristics::is_in_truck_tour(int id, deque <int> &given_tour){
    deque<int>::iterator it;  //An iterator on int list
    bool is_in = false;

    for(it = given_tour.begin(); it!=given_tour.end(); ++it){
        if(*it==id){
            is_in = true;
            break;
        }
    }
    return is_in;
}

//Returns true if id is found droneEligible list
bool Heuristics::is_drone_eligible(int id){
    deque<int>::iterator it;  //An iterator on int list
    bool is_in = false;

    for(it = droneEligible.begin(); it!=droneEligible.end(); ++it){
        if((*it)==id){
            is_in = true;
            break;
        }
    }
    return is_in;
}

//Laurent proposition
void Heuristics::new_giant_tour_construction_laurent (deque<int> &truck_tour, deque < deque<int> > &tours, deque < deque<int> > &drones){

    deque<int> values;
    int nb;
    bool check;

    deque<deque <int> >::iterator it2;
    deque<int>::iterator it3;  //An iterator on int l

    /******* Concatenate vehicle's tours randomly *******/

    truck_tour.clear();
    values.clear();
    for (int i=0;i<NB_VEH;i++)
    {
        do
        {
            nb=rand()%NB_VEH;
            //check or number is already used:
            check=true;
            for (int j=0;j<i;j++)
                if (nb == values[j]) //if number is already used
                {
                    check=false; //set check to false
                    break; //no need to check the other elements of value[]
                }
        } while (!check); //loop until new, unique number is found
        values.push_back(nb); //store the generated number in the array
    }

    //Verification
    cout<<"*** Ordre de concatenation ***"<<endl;
    for(int i=0; i< values.size(); i++)
        cout<<values[i]<<";";
    cout<<endl;

    //concatenation
    copy_deque_to_deque(tours[values[0]],truck_tour);
    for(int i=1; i<values.size(); i++){
        concate(truck_tour,tours[values[i]]);
    }

    //Verification
    /*cout<<"***** Tour vehicule apres concatenation *****"<<endl;
    for(it3 = truck_tour.begin(); it3!=truck_tour.end(); ++it3){
        cout<<*it3<<";";
    }
    cout<<endl;
    cout<<"***** liste drones Apres MOVES*****"<<endl;
    for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
        for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
            cout<< *it3<<",";
        }
        cout<<endl;
    }*/


    /******* Best insertion *******/

    /*for(it2 = best_drone_assignment.begin(); it2!=best_drone_assignment.end(); ++it2){
        for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
            best_insertion(truck_tour, *it3);
        }
    }*/

    /********* Random Insertion *******/
    for(it2 = drones.begin(); it2!=drones.end(); ++it2){
        for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
            random_insertion(truck_tour, *it3);
        }
    }
    cout<<"taille truck_tour après random insertion = "<<truck_tour.size()<<endl;
    cout<<"cout truck_tour avant 2-opt = "<<get_truck_cost(truck_tour)<<endl;

    /****** 2-opt ******/

    truck_tour.pop_back();
    amelioration();

    cout<<"taille truck_tour après 2-opt = "<<truck_tour.size()<<endl;
    cout<<"cout truck_tour apres 2-opt = "<<get_truck_cost(truck_tour)<<endl;

}

//Laurent proposition
void Heuristics::new_giant_tour_construction_dominique(deque<int> &truck_tour, deque < deque<int> > &tours, deque < deque<int> > &drones, int X){

    deque<int> values, lot;
    int nb, taille;
    bool check;

    deque<deque <int> >::iterator it2;
    deque<int>::iterator it3;  //An iterator on int l

    cout<<"X = "<<X<<endl;
    truck_tour.clear();
    values.clear();

    /******* Concatenate X first vehicle's tours randomly *******/
    if(X!=0){
        for (int i=0;i<X;i++)
        {
            do
            {
                nb=rand()%X;
                //check or number is already used:
                check=true;
                for (int j=0;j<i;j++)
                    if (nb == values[j]) //if number is already used
                    {
                        check=false; //set check to false
                        break; //no need to check the other elements of value[]
                    }
            } while (!check); //loop until new, unique number is found
            values.push_back(nb); //store the generated number in the array
        }

        //Verification
        cout<<" Ordre de concatenation des tours vehicule : ";
        for(int i=0; i< values.size(); i++)
            cout<<values[i]+1<<";";
        cout<<endl;

        //concatenation
        copy_deque_to_deque(tours[values[0]],truck_tour);
        for(int i=1; i<values.size(); i++){
            concate(truck_tour,tours[values[i]]);
        }

        //Verification
        /*cout<<"***** Tour vehicule apres concatenation *****"<<endl;
        for(it3 = truck_tour.begin(); it3!=truck_tour.end(); ++it3){
            cout<<*it3<<";";
        }
        cout<<endl;
        cout<<"***** liste drones Apres MOVES*****"<<endl;
        for(it2 = drone_assignment.begin(); it2!=drone_assignment.end(); ++it2){
            for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
                cout<< *it3<<",";
            }
            cout<<endl;
        }*/
        //cout<<"taille du vehicule concatene "<<truck_tour.size();
        }else{
            truck_tour.push_back(0);
            truck_tour.push_back(0);
        }
    /******* Best insertion random *******/

    //put all remaining vehicle customers and drone customers in lot
    lot.clear();
    for(it2 = tours.begin()+X; it2!=tours.end(); ++it2){
        for (it3 = (*it2).begin()+1; it3!=(*it2).end()-1; ++it3)
            lot.push_back(*it3);
    }

    for(it2 = drones.begin(); it2!=drones.end(); ++it2){
        for(it3 = (*it2).begin(); it3!=(*it2).end(); ++it3){
            lot.push_back(*it3);
        }
    }

    //cout<<"On arrive ici"<<endl;
    //best insertion of customers in lot chosen randomly

    taille = lot.size();
    //cout<<"Taille de lot = "<<taille<<endl;
    values.clear();

    for (int i=0;i<taille;i++)
    {
        do
        {
            nb=rand()%taille; //generate random number between 0 and taille-1
            //check or number is already used:
            check=true;
            for (int j=0;j<i;j++)
                if (nb == values[j]) //if number is already used
                {
                    check=false; //set check to false
                    break; //no need to check the other elements of value[]
                }
        } while (!check); //loop until new, unique number is found
        values.push_back(nb); //store the generated number in the array
    }

    for(int i=0; i<values.size(); i++)
        best_insertion(truck_tour,lot[values[i]]);


    //cout<<"taille truck_tour apres best insertion random = "<<truck_tour.size()<<endl;
    //cout<<"cout truck_tour apres best insertion random = "<<get_truck_cost(truck_tour)<<endl;

}

//Best insertion of id in truck_tour
void Heuristics::best_insertion(deque<int> &given_tour,int id){
    double current_cost, best_cost;

    int best_index;
    std::deque<int>::iterator it = given_tour.begin();
    it++;
    best_index = 1;
    it = given_tour.insert(it, id); //we insert id at position 1 (just after the origin depot)

    current_cost = get_truck_cost(given_tour);
    best_cost = current_cost;

    given_tour.erase(it);//we delete id that we just previously added

    int j = 1;

    while(it < given_tour.end()-1){
        it = given_tour.begin()+j;
        it = given_tour.insert(it,id); //we insert id at position j
        current_cost = get_truck_cost(given_tour);
        if(current_cost < best_cost){
            best_cost = current_cost;
            best_index = j;
        }

        given_tour.erase(it);//we delete id that we just previously added

        j++;
    }

    //we insert id at best_index
    //if(best_index == truck_tour.begin())
     //   best_index = truck_tour.begin()+1;

    given_tour.insert(given_tour.begin()+best_index, id);
}


//Random insertion of id in truck_tour
void Heuristics::random_insertion(deque<int> &given_tour,int id){

    int index = rand()%(given_tour.size()-2) + 1;
    //cout<<endl;
    //cout<<"Random Index "<<index<<endl;
    //cout<<endl;
    given_tour.insert(given_tour.begin()+index, id);
}


//Finds cost related to given_tour
double Heuristics::get_truck_cost(deque <int> &given_tour){
    double cost = 0;
    deque<int>::iterator it;  //An iterator on int list

    for(it = given_tour.begin(); it!=given_tour.end()-1; ++it)
    {
        cost += matrixTruck[*it][*(it+1)];
    }

    return cost;
}

//Finds cost related to given_drone_list
double Heuristics::get_drone_cost(deque <int> & given_drone_list){
    double cost = 0;
    deque<int>::iterator it;  //An iterator on int list

    for(it = given_drone_list.begin(); it!=given_drone_list.end(); ++it)
    {
        cost += matrixDrone[0][*it]*2;
    }

    return cost;
}

//Returns worst vehicle cost
double Heuristics::get_worst_veh_cost(deque < deque<int> > &tours){
    double worstCost, cost;
    int n = tours.size(),i=1;
    worstCost = get_truck_cost(tours[0]);
    while(i<n){
        cost = get_truck_cost(tours[i]);
        if(cost>worstCost)
            worstCost = cost;
        i++;
    }
    return worstCost;
}
//Returns worst vehicle cost
double Heuristics::get_worst_drone_cost(deque < deque<int> > &drones){
    double worstCost, cost;
    int n = drones.size(),i=1;
    worstCost = get_drone_cost(drones[0]);

    while(i<n){
        cost = get_drone_cost(drones[i]);
        if(cost>worstCost)
            worstCost = cost;
        i++;
    }
    return worstCost;
}

//Returns true if l1 is better than l2
bool Heuristics::compare_label(Label *l1, Label *l2){

    if(!double_equals(max(l1->c1,l1->c2), max(l2->c1,l2->c2)) && (max(l1->c1,l1->c2) < max(l2->c1,l2->c2))){
       return true;
    }else
        if(double_equals(max(l1->c1,l1->c2), max(l2->c1,l2->c2)) && (min(l1->c1,l1->c2) < min(l2->c1,l2->c2))){
            return true;
        }else
            return false;
}


//Returns true if l1 is better than l2
bool Heuristics::compare_label2(Label2 *l1, Label2 *l2){

    if(!double_equals(maxi(l1->c_gloVeh,l1->c1,l1->c2), maxi(l2->c_gloVeh,l2->c1,l2->c2)) &&
       (maxi(l1->c_gloVeh,l1->c1,l1->c2) < maxi(l2->c_gloVeh,l2->c1,l2->c2))){
       return true;
    }else
        if(double_equals(maxi(l1->c_gloVeh,l1->c1,l1->c2), maxi(l2->c_gloVeh,l2->c1,l2->c2)) &&
           (min(l1->c1,l1->c2) < min(l2->c1,l2->c2))){
            return true;
        }else
            return false;
}

//Returns true if current solution is better than recor solution
bool Heuristics::compare_solutions1(Label *l1, Label *l2, deque < deque<int> > &Veh, deque < deque<int> > &Dro, deque < deque<int> > &bestVeh, deque < deque<int> > &bestDro){

    double sum1_S=0, sum2_S=0, sum1_S_prim=0, sum2_S_prim=0;
    deque<deque <int> >::iterator it2;

    if(!double_equals(max(l1->c1,l1->c2), max(l2->c1,l2->c2)) && (max(l1->c1,l1->c2) < max(l2->c1,l2->c2))){
       return true;
    }else{
        for(it2 = Veh.begin(); it2!=Veh.end(); ++it2){
            sum1_S = sum1_S + get_truck_cost(*it2);
        }
        for(it2 = bestVeh.begin(); it2!=bestVeh.end(); ++it2){
            sum1_S_prim = sum1_S_prim + get_truck_cost(*it2);
        }

        for(it2 = Dro.begin(); it2!=Dro.end(); ++it2){
            sum2_S = sum2_S + get_drone_cost(*it2);
        }
        for(it2 = bestDro.begin(); it2!=bestDro.end(); ++it2){
            sum2_S_prim = sum2_S_prim + get_drone_cost(*it2);
        }

        if(double_equals(max(l1->c1,l1->c2), max(l2->c1,l2->c2)) &&
           sum1_S+sum2_S < sum1_S_prim + sum2_S_prim){
           return true;
        }else
            return false;
    }
}

//Returns true if current solution is better than recor solution
bool Heuristics::compare_solutions2(Label *l1, Label *l2, deque < deque<int> > &Veh, deque < deque<int> > &Dro, deque < deque<int> > &bestVeh, deque < deque<int> > &bestDro){

    double sum1_S=0, sum2_S=0, sum1_S_prim=0, sum2_S_prim=0;
    deque<deque <int> >::iterator it2;

    if(!double_equals(max(l1->c1,l1->c2), max(l2->c1,l2->c2)) && (max(l1->c1,l1->c2) < max(l2->c1,l2->c2))){
       return true;
    }else{
        for(it2 = Veh.begin(); it2!=Veh.end(); ++it2){
            sum1_S = sum1_S + pow(max(l1->c1,l1->c2) - get_truck_cost(*it2), 2);
        }
        for(it2 = bestVeh.begin(); it2!=bestVeh.end(); ++it2){
            sum1_S_prim = sum1_S_prim + pow(max(l2->c1,l2->c2) - get_truck_cost(*it2), 2);
        }

        for(it2 = Dro.begin(); it2!=Dro.end(); ++it2){
            sum2_S = sum2_S + pow(max(l1->c1,l1->c2) - get_drone_cost(*it2), 2);
        }
        for(it2 = bestDro.begin(); it2!=bestDro.end(); ++it2){
            sum2_S_prim = sum2_S_prim + pow(max(l2->c1,l2->c2) - get_drone_cost(*it2), 2);
        }

        if(double_equals(max(l1->c1,l1->c2), max(l2->c1,l2->c2)) &&
           sum1_S+sum2_S > sum1_S_prim + sum2_S_prim){
           return true;
        }else
            return false;
    }
}
//Sort drone_list according to the distance to the depot
void Heuristics::sortDroneDeque(){

    int n;
    int taille = drone_list.size();
    for(int i=taille-1; i>0; i--){
        for(int j=0; j<i; j++){
            if ((matrixDrone[0][drone_list[j]])*2 < (matrixDrone[0][drone_list[j+1]])*2){
                n = drone_list[j];
                drone_list[j] = drone_list[j+1];
                drone_list[j+1] = n;
            }
        }
    }
}

//Find the drone with the smallest ending time
int Heuristics::findSmallestDroneTime(double endingTimes[]){
    int smallest = endingTimes[0] ;
    int index = 0;
    for (int i=1;  i < NB_DRONE; i++)
        if (endingTimes[i] < smallest ){
             smallest = endingTimes[i] ;
             index = i;
        }
    return index;
}

//Returns yes is a is equal to b
bool Heuristics::double_equals(double a, double b)
{
    double epsilon = 0.001;
    return fabs(a - b) < epsilon;
}

//Find the best label in a labels list L
Label* Heuristics::find_best_label(deque <Label*> L){

    Label *best_label;
    best_label = L[0];
    deque<Label*>::iterator it;  //An iterator on Labels list

    for(it = L.begin(); it!=L.end(); ++it){
        if (compare_label((*it),best_label)){
            best_label = (*it);
        }
    }

    return best_label;
}

//Find the best label in a labels list L
Label2* Heuristics::find_best_label2(deque <Label2*> L){

    Label2 *best_label;
    best_label = L[0];
    deque<Label2*>::iterator it;  //An iterator on Labels list

    for(it = L.begin(); it!=L.end(); ++it){
        if (compare_label2((*it),best_label)){
            best_label = (*it);
        }
    }

    return best_label;
}


//Find the 3 best labels in a labels list L
deque<Label2*> Heuristics::find_best_label3(deque <Label2*> &L){

    Label2 *lab1,*lab2,*lab3;
    deque<Label2*> bestLabels;

    lab1 = NULL;
    lab2 = NULL;
    lab3 = NULL;

    deque<Label2*>::iterator it;  //An iterator on Labels list

    int taille = L.size();
    //cout<<"La taille de L est "<<taille<<endl;
    /*if(taille<=3){
        cout<<"Voici les labels de la liste L"<<endl;
        for(it=L.begin(); it=L.end(); it++)
            cout<<"("<<(*it)->c_gloVeh<<","<<(*it)->c1<<","<<(*it)->c2<<","<<(*it)->numVeh<<");";
        cout<<endl;
    }*/


    if(taille>=3){
        if(compare_label2(L[0],L[1])){
            lab1 = L[0];
            lab2 = L[1];
        }else{
            if(compare_label2(L[1],L[0])){
                lab1 = L[1];
                lab2 = L[0];
            }
        }
        if(compare_label2(L[2],lab1)){
            lab3 = lab2;
            lab2 = lab1;
            lab1 = L[2];
        }else{
            if(compare_label2(lab1,L[2]) && compare_label2(L[2],lab2)){
                lab3 = lab2;
                lab2 = L[2];
            }else{
                lab3 = L[2];
            }
        }

        if(taille>3){
            it = L.begin()+3;
            while(it != L.end()){
                if(compare_label2(*it,lab1)){
                    lab3 = lab2;
                    lab2 = lab1;
                    lab1 = *it;
                }else{
                    if(compare_label2(lab1,*it) && compare_label2(*it,lab2)){
                        lab3 = lab2;
                        lab2 = *it;
                    }else{
                        if(compare_label2(*it,lab3))
                            lab3 = *it;
                    }
                }
                it++;
            }
        }
    }else{
        if(taille == 2){
            if(compare_label2(L[0],L[1])){
                lab1 = L[0];
                lab2 = L[1];
            }else{
                if(compare_label2(L[1],L[0])){
                    lab1 = L[1];
                    lab2 = L[0];
                }
            }
        }else{
          lab1 = L[0];
        }
    }
    //verification
    cout<<"Voici les 3 meilleurs labels de L : "<<endl;
    if(lab1 != NULL)
        cout<<"lab1 = ("<<lab1->c_gloVeh<<","<<lab1->c1<<","<<lab1->c2<<","<<lab1->numVeh<<")"<<endl;
    if(lab2 != NULL)
        cout<<"lab2 = ("<<lab2->c_gloVeh<<","<<lab2->c1<<","<<lab2->c2<<","<<lab2->numVeh<<")"<<endl;
    if(lab3 != NULL)
        cout<<"lab3 = ("<<lab3->c_gloVeh<<","<<lab3->c1<<","<<lab3->c2<<","<<lab3->numVeh<<")"<<endl;


    bestLabels.clear();
    if(lab1 != NULL)
        bestLabels.push_back(lab1);
    if(lab2 != NULL)
        bestLabels.push_back(lab2);
    if(lab3 != NULL)
        bestLabels.push_back(lab3);

    return bestLabels;
}


//Returns the direct successor of i in tour
int Heuristics::succ(deque<int> &tour, int i){
    bool trouve = false;
    int suc = -1;
    deque<int>::iterator it;
    it = tour.begin();
    while(it<tour.end()-1 && !trouve){
        if(*it == i)
            trouve = true;
        else
            it++;
    }
    if(trouve)
        suc = *(it+1);

    return suc;

}

//Returns the position index of i in tour
int Heuristics::getIndex(deque<int> &tour, int i){
    bool trouve = false;
    int taille = tour.size(), index = 0;

    while(index < taille && !trouve)
        if(tour[index] == i)
            trouve = true;
        else
            index++;

    if(!trouve)
        index = -1;

    return index;
}

//Stat analysis of impact of the quality of the TSP tour
void Heuristics::statAnalysis(string path, string file_n){
    double cost1, min_cost;
    int nb_iteration=1, index;
    tsp *giant_tsp;
    Label *l1,*l2;
    deque<int>::iterator it3;

    deque<int> T[21];
    deque<Node*> Tours[21];
    double costs[21];

    deque <Node*> tour1, tour2;

    string path1 = path+"/statAnal.csv", path2,str;
    const char *filename = path1.c_str();

    //cout<<"on entre ici"<<endl;


    giant_tsp = new tsp(all_customers); //creates a tsp object

    costs[0] = giant_tsp->run_tsp(0); //the result is found in solution deque
    T[0] = giant_tsp->solution;
    min_cost = costs[0];
    index = 0;
    //cout<<"costs[0] = "<<costs[0]<<endl;

    for(it3=giant_tsp->solution.begin(); it3!=giant_tsp->solution.end(); ++it3)
        Tours[0].push_back(new Node(*find_node_by_id(*it3)));
    //we add the depot at the end of tour
    Tours[0][0]->set_drone(1); //the origin depot node is imposed to the truck
    Tours[0].push_back(new Node(*Tours[0][0]));


    //cout<<"on arrive ici"<<endl;
    delete giant_tsp;

    for (int i=1; i<21; i++){

        /******* Step 1 : build a giant TSP *******/
        giant_tsp = new tsp(all_customers);
        costs[i] = giant_tsp->run_tsp(i); //the result is found in solution deque
        T[i] = giant_tsp->solution;

        //cout<<"costs["<<i<<"] = "<<costs[i]<<endl;

        for(it3=giant_tsp->solution.begin(); it3!=giant_tsp->solution.end(); ++it3)
            Tours[i].push_back(new Node(*find_node_by_id(*it3)));
        //we add the depot at the end of tour
        Tours[i][0]->set_drone(1); //the origin depot node is imposed to the truck
        Tours[i].push_back(new Node(*Tours[i][0]));

        if(costs[i] < min_cost){
            min_cost = costs[i];
            index = i;
        }
        delete giant_tsp;
    }

    cost1 = min_cost;
    //cout<<"index = "<<index<<endl;

    /******* Step 2 : apply Split procedure on tour  *******/

    l1 = split(Tours[index],nb_iteration);
    //cout <<"cout l1 apres split = "<<max(l1->c1,l1->c2)<<endl;
    for(int i=0; i<21; i++){
        if(i != index){
            l2 = split(Tours[i],nb_iteration);
            //cout <<"cout l2 apres split = "<<max(l2->c1,l2->c2)<<endl;

            //write result in file
            ofstream write(filename,ios::app);
            if (write.is_open())
            {
                write << file_n <<";"<< floor(((costs[i]/cost1) - 1)*100)<<";"<<floor(((max(l2->c1,l2->c2)/max(l1->c1,l1->c2)) - 1)*100)<< '\n';
            }

            //cout<<"alpha = "<<((costs[i]/cost1) - 1)*100<<endl;
            //cout<<"beta = "<<((max(l2->c1,l2->c2)/max(l1->c1,l1->c2)) - 1)*100<<endl;
            //cout<<endl;
        }
    }
}

void Heuristics::limitLabelAnalysis(string path, string file_n){

    tsp *giant_tsp;
    Label2 *l, *l1, *l2, *l3;
    deque<int>::iterator it;
    deque<Node*>::iterator it2;
    deque<Node*> T;
    int nbLab,nbLab1,nbLab2,nbLab3;
    double err1, err2, err3, v, v1, v2, v3;
    //string path1;
    //const char *filename1, *filename2, *filename3;

    string filename[3] = {path+"/limitLab1.csv", path+"/limitLab2.csv", path+"/limitLab3.csv"};

   /* path1 = path+"/limitLab1.csv";
    filename1 = path1.c_str();

    path1 = path+"/limitLab2.csv";
    filename2 = path1.c_str();

    path1 = path+"/limitLab3.csv";
    filename3 = path1.c_str();
*/
    //cout<<"size(all_customers) = "<<all_customers.size()<<endl;


    for (int i=0; i<25; i++){

        cout<<endl;
        /******* Step 1 : build a giant TSP *******/
        giant_tsp = new tsp(all_customers);
        giant_tsp->run_tsp(i);

        for(it=giant_tsp->solution.begin(); it!=giant_tsp->solution.end(); ++it)
            T.push_back(new Node(*find_node_by_id(*it)));

        //we add the depot at the end of tour
        T[0]->set_drone(1); //the origin depot node is imposed to the truck
        T.push_back(new Node(*T[0]));


        cout<<"*** No Limitation ****"<<endl;
        l = split2(T,1,0);//split without limitation of labels
        v = maxi(l->c_gloVeh, l->c1, l->c2);
        nbLab = nbGenLab;
        //cout<<"maxi(l->c_gloVeh, l->c1, l->c2) = "<<maxi(l->c_gloVeh, l->c1, l->c2)<<endl;
        cout<<"nbLab = "<<nbLab<<endl;

        cout<<"*** filter1 choix 1 ****"<<endl;
        l1 = split2(T,1,1);//split with limitation of labels filter1 (sum)
        v1 = maxi(l1->c_gloVeh, l1->c1, l1->c2);
        nbLab1 = nbGenLab;
        err1 = ((v1-v)/v)*100;
        cout<<"maxi(l1->c_gloVeh, l1->c1, l1->c2) = "<<maxi(l1->c_gloVeh, l1->c1, l1->c2)<<endl;
        cout<<"nbLab1 = "<<nbLab1<<" ; err1 = "<<err1<<endl;


        cout<<"*** filter1 choix 2 ****"<<endl;
        l2 = split2(T,1,2);//split with limitation of labels filter1 (max)
        v2 = maxi(l2->c_gloVeh, l2->c1, l2->c2);
        nbLab2 = nbGenLab;
        err2 = ((v2-v)/v)*100;
        //cout<<"maxi(l2->c_gloVeh, l2->c1, l2->c2) = "<<maxi(l2->c_gloVeh, l2->c1, l2->c2)<<endl;
        cout<<"nbLab2 = "<<nbLab2<<" ; err2 = "<<err2<<endl;

        cout<<"*** filter2 ****"<<endl;
        l3 = split2(T,1,3);//split with limitation of labels filter2
        v3 = maxi(l3->c_gloVeh, l3->c1, l3->c2);
        nbLab3 = nbGenLab;
        err3 = ((v3-v)/v)*100;
        //cout<<"maxi(l3->c_gloVeh, l3->c1, l3->c2) = "<<maxi(l3->c_gloVeh, l3->c1, l3->c2)<<endl;
        cout<<"nbLab3 = "<<nbLab3<<" ; err3 = "<<err3<<endl;

        //write results in files
        ofstream outputFile[3];

        for (int i = 0; i < 3; i++)
        {
            outputFile[i].open(filename[i].c_str(), ios::app);
        }

        outputFile[0] << file_n <<";"<< nbLab <<";"<< nbLab1 <<";"<< err1 << '\n';
        outputFile[1] << file_n <<";"<< nbLab <<";"<< nbLab2 <<";"<< err2 << '\n';
        outputFile[2] << file_n <<";"<< nbLab <<";"<< nbLab3 <<";"<< err3 << '\n';

        for (int i = 0; i < 3; i++)
        {
            outputFile[i].close();
        }

        /*ofstream write1(filename1,ios::app);
        if (write1.is_open())
        {
            write1 << file_n <<";"<< nbLab <<";"<< nbLab1 <<";"<< err1 << '\n';
        }
        ofstream write2(filename2,ios::app);
        if (write1.is_open())
        {
            write2 << file_n <<";"<< nbLab <<";"<< nbLab2 <<";"<< err2 << '\n';
        }
        ofstream write3(filename3,ios::app);
        if (write3.is_open())
        {
            write3 << file_n <<";"<< nbLab <<";"<< nbLab3 <<";"<< err3 << '\n';
        }
        */
        for(it2=T.begin(); it2!=T.end(); ++it2)
           delete *it2;
        T.clear();

        delete giant_tsp;
    }

}

//Returns the max between a, b and c
double Heuristics::maxi(double a, double b, double c){
    double val;
    if(a>b)
        val=a;
    else
        val=b;
    if (c>val)
        val=c;
    return val;
}

//Transforms a table in deque of deque
deque< deque <int> > Heuristics::transformed(deque<int>* &tab){
    deque< deque <int> > result;

    for(int i=0; i<NB_VEH; i++){
        result.push_back(tab[i]);
    }

    return result;
}


//Copies a deque of int from source to dest
void Heuristics::copy_deque_to_deque(deque <int> & source, deque <int> & dest)
{
    deque<int>::iterator it;  //An iterator on Labels list

    dest.clear();

    for(it = source.begin(); it!=source.end(); ++it)
        dest.push_back(*it);
}

//Copies a table of deque from source to dest
void Heuristics::copy_dequetab_to_dequetab(deque< deque <int> > &source, deque< deque <int> > &dest)
{
    deque<deque <int> >::iterator it;
    deque<int>::iterator it2;

    int length = source.size();
    deque<int> deques[length]; //table of list (each list will contain the set of customer id associated to the given drone)

    //we make sure dest is empty
    for(it = dest.begin(); it!=dest.end(); ++it)
        (*it).clear();
    dest.clear();

    for(it = source.begin(); it!=source.end(); ++it){
        for(it2 = (*it).begin(); it2!=(*it).end(); ++it2)
            deques[std::distance(source.begin(), it)].push_back(*it2);
    }

    for(int i=0; i<length; i++)
        dest.push_back(deques[i]);
}

//Concatenates two vehicle tours (in this order tour1-tour2)
void Heuristics::concate(deque<int> &tour1, deque<int> &tour2)
{
    deque<int>::iterator it;

    tour1.pop_back();
    it = tour2.begin()+1;

    while (it != tour2.end()){
        tour1.push_back(*it);
        ++it;
    }
}


//Writes the result of the heuristic into file_name
void Heuristics::write_result(const char * file_name,const char * file_n, Label *best_label, int best_nb_truck_customers, int best_nb_drone_customers, double time,double timeSplit, double timeOptimize, double timeLocalSearch,int nb_restart, int nbSplitCall,int averageNbGenLab, double percentDelLab, double averageGapDrone){
    ofstream write(file_name,ios::app);
/*    string s,d;
    stringstream p; //nb;
    deque<int>::iterator it;
    int taille = final_trucks_tours.size();
    for(int i=0; i<taille; i++){
        s = s+"0";
        it = final_trucks_tours[i].begin()+1;
        while(it != final_trucks_tours[i].end()){

           p.str("");
           p<<*it;
           s = s+"->"+p.str();
           it++;
        }
        s = s+";";
    }

    taille = final_drone_assignment.size();
    cout<<"taille final_drone_assignment = "<<taille<<endl;

    for(int i=0; i<taille; i++){
        if(final_drone_assignment[i].size()==0){
            p.str("/");
            d = d+p.str();
            d = d+";";
        }else{
            it = final_drone_assignment[i].begin();
            p.str("");
            p<<*it;
            d = d+p.str();
            it++;
            while(it != final_drone_assignment[i].end()){

               p.str("");
               p<<*it;
               d = d+","+p.str();
               it++;
            }
            d = d+";";
        }
    }

    if(taille < NB_DRONE){
        for(int i=NB_DRONE;i>taille;i--)
            d = d+"/;";
    }
*/
    if (write.is_open())
    {
        //write <<"Instance;Nbre client vehicule;Nbre client drone;Coût véhicule; Coût drone; Temps d'exécution; Nbre de démarrage; Nbre moyen de split; Nbre moyen labels générés; Pourcentage label supprimés; Gap LPT vs SPLIT "<< '\n';
        write << file_n <<";"<< droneEligible.size()<<";"<< best_nb_truck_customers <<";"<< best_nb_drone_customers <<";"<< best_label->c1<<";"<<best_label->c2<<";"<<time <<";"<<timeSplit<<";"<<timeOptimize <<";"<<timeLocalSearch <<";"<<nb_restart<<";"<<nbSplitCall<<";"<<averageNbGenLab<<";"<<percentDelLab<<";"<<averageGapDrone<< '\n';
    }
}

//Writes the result of the ILS procedure into file_name
void Heuristics::write_result_ILS(const char * file_name, const char * file_n, Label *best_label, int best_nb_truck_customers, int best_nb_drone_customers, double time,double timeSplit, double timeOptimize, double timeLocalSearch, int nbSplitCall,int averageNbGenLab,double percentDelLab, double averageGapDrone){
    ofstream write(file_name,ios::app);
    /*string s,d;
    stringstream p; //nb;
    deque<int>::iterator it;
    int taille = best_trucks_tours.size();
    for(int i=0; i<taille; i++){
         s = s+"0";
        it = best_trucks_tours[i].begin()+1;
        while(it != best_trucks_tours[i].end()){

           p.str("");
           p<<*it;
           //cout<<"p = "<<p.str()<<endl;
           s = s+"->"+p.str();
           it++;
        }
        s = s+";";
    }

    //cout<<"s = "<<s<<endl;
    taille = best_drone_assignment.size();
    cout<<"taille final_drone_assignment = "<<taille<<endl;
    //cout<< final_drone_assignment[1][0]<<endl;

    for(int i=0; i<taille; i++){
        if(best_drone_assignment[i].size()==0){
            p.str("/");
            d = d+p.str();
            d = d+";";
        }else{
            it = best_drone_assignment[i].begin();
            p.str("");
            p<<*it;
            d = d+p.str();
            it++;
            while(it != best_drone_assignment[i].end()){

               p.str("");
               p<<*it;
               //cout<<"p = "<<p.str()<<endl;
               d = d+","+p.str();
               it++;
            }
            d = d+";";
        }
    }
    //cout<<"d = "<<d<<endl;

    if(taille < NB_DRONE){
        for(int i=NB_DRONE;i>taille;i--)
            d = d+"/;";
    }
    */
    if (write.is_open())
    {
        //write <<"Instance;Nbre client vehicule;Nbre client drone;Coût véhicule; Coût drone; Temps d'exécution; Nbre de démarrage; Nbre moyen de split; Nbre moyen labels générés; Pourcentage label supprimés; Gap LPT vs SPLIT "<< '\n';
        write << file_n <<";"<< droneEligible.size()<<";"<< best_nb_truck_customers <<";"<< best_nb_drone_customers <<";"<< best_label->c1<<";"<<best_label->c2<<";"<<time <<";"<<timeSplit<<";"<<timeOptimize <<";"<<timeLocalSearch <<";"<<nbSplitCall<<";"<<averageNbGenLab<<";"<<percentDelLab<<";"<<averageGapDrone<< '\n';
    }
}

//Writes the result of the ILS procedure into file_name
void Heuristics::write_result_ILS2(const char * file_name, const char * file_n, Label *best_label, int best_nb_truck_customers, int best_nb_drone_customers, double time,double timeSplit, double timeOptimize, double timeLocalSearch, int nbSplitCall,int averageNbGenLab,double percentDelLab, double averageGapDrone){
    ofstream write(file_name,ios::app);

    if (write.is_open())
    {
        //write <<"Instance;Nbre client vehicule;Nbre client drone;Coût véhicule; Coût drone; Temps d'exécution; Nbre de démarrage; Nbre moyen de split; Nbre moyen labels générés; Pourcentage label supprimés; Gap LPT vs SPLIT "<< '\n';
        write << file_n <<";"<<best_nb_truck_customers <<";"<< best_nb_drone_customers <<";"<< best_label->c1<<";"<<best_label->c2<<";"<<time <<";"<<timeSplit<<";"<<timeOptimize <<";"<<timeLocalSearch <<";"<<nbSplitCall<<";"<<averageNbGenLab<<";"<<percentDelLab<<";"<<averageGapDrone<< '\n';
    }


}

