//compile with g++ quadtree1.cpp, with airfoil-points.txt in the same directory
#include<iostream>
#include<new>
#include<vector>
#include<iomanip>
#include<fstream>
#include<cmath>

using namespace std;

int maxLevel=0; //just to store level of farthest leaf node.
                //better than writing a whole new recursive function.
                //tbh makes the code a bit ugly
//int preBlankedPoints = 0;
struct quadNode{
    struct quadNode *nw;
    struct quadNode *ne;
    struct quadNode *sw;
    struct quadNode *se;
    float x1, x2, y1, y2;
    int level;
    struct quadNode *parent;
    vector<struct quadNode *> neighbor[8];
    //vector<float> neighborLevel;
    vector<float> boundaryXcord;//contains the coordinates of the orignal boundary points,
    vector<float> boundaryYcord;//so that they can be passed to the child nodes. Root nodes has all the points.
};

void newLevel(struct quadNode* , float , float , float , float );
bool liesIn(struct quadNode *node, float x, float y);
void constructTree(struct quadNode *parent);

bool liesIn(struct quadNode *node, float x, float y){//should be modified to deal with points
    if( node->x1 < x && node->y1 < y &&              //lying on the boundary of quadrants formed.
        node->x2 > x && node->y2 > y)
        return true;
    else 
        return false;
}

void constructTree(struct quadNode *parent){//function that makes the basic tree with the orignal boundary points.
    int numBndPoints = parent->boundaryXcord.size();
    if(numBndPoints > 1){
        newLevel(parent, parent->x1, parent->y1, parent->x2, parent->y2);
    }
    if(maxLevel<parent->level)//updates the max level created
        maxLevel = parent->level;
}

void newLevel(struct quadNode *parent, float x1, float y1, float x2, float y2){//x and y are boundaries of parent
    struct quadNode *nw = new struct quadNode;
    struct quadNode *ne = new struct quadNode;
    struct quadNode *sw = new struct quadNode;
    struct quadNode *se = new struct quadNode;

    nw->nw = nw->ne = nw->sw = nw->se = NULL;
    ne->nw = ne->ne = ne->sw = ne->se = NULL;
    sw->nw = sw->ne = sw->sw = se->se = NULL;
    se->nw = se->ne = se->sw = se->se = NULL;

    nw->parent = ne->parent = sw->parent = se->parent = parent;

    nw->x1 = x1;        
    nw->y1 = (float)(y1+y2)/2;
    nw->x2 = (float)(x1+x2)/2;
    nw->y2 = y2;

    ne->x1 = (float)(x1+x2)/2;
    ne->y1 = (float)(y1+y2)/2;
    ne->x2 = x2;
    ne->y2 = y2;

    sw->x1 = x1;
    sw->y1 = y1;
    sw->x2 = (float)(x1+x2)/2;
    sw->y2 = (float)(y1+y2)/2;

    se->x1 = (float)(x1+x2)/2;
    se->y1 = y1;
    se->x2 = x2;
    se->y2 = (float)(y1+y2)/2;

    nw->level = ne->level = sw->level = se->level = parent->level + 1;

    int numBndPoints = parent->boundaryXcord.size();
    if(numBndPoints > 1){
        for(int i=0; i<numBndPoints; i++){
            float x = parent->boundaryXcord[i];
            float y = parent->boundaryYcord[i];
            if(liesIn(nw, x, y)){
                nw->boundaryXcord.push_back(x);
                nw->boundaryYcord.push_back(y);
            }
            else if(liesIn(ne, x, y)){
                ne->boundaryXcord.push_back(x);
                ne->boundaryYcord.push_back(y);
            }
            else if(liesIn(sw, x, y)){
                sw->boundaryXcord.push_back(x);
                sw->boundaryYcord.push_back(y);
            }
           else if(liesIn(se, x, y)){
                se->boundaryXcord.push_back(x);
                se->boundaryYcord.push_back(y);
            }
        }
    }

    constructTree(nw);
    constructTree(ne);
    constructTree(sw);
    constructTree(se);

    parent->nw = nw;
    parent->ne = ne;
    parent->sw = sw;
    parent->se = se;

    return;
}
//Traverse function are required because many times the neighbors 
//have further child nodes that actually give the neighbor info
void eastTraverse(struct quadNode *node, vector<struct quadNode *> *neighbor){
    if(node->ne != NULL){//any child can be checked to find if child exists
        eastTraverse(node->ne, neighbor);
        eastTraverse(node->se, neighbor);
    }

    neighbor[6].push_back(node);
}

void eastNeighbor(struct quadNode *node){
    vector<string> path;
    struct quadNode *tempNode = node;
    struct quadNode *tempParent;
    if(tempNode->parent == NULL){
        node->neighbor[6].push_back(NULL);
        return;
    }
    else
        tempParent = tempNode->parent;
    
    
    while(tempNode == tempParent->ne || tempNode == tempParent->se ){
        if(tempNode == tempParent->ne)
            path.push_back("ne");
        else if(tempNode == tempParent->se)
            path.push_back("se");
        tempNode = tempParent;
        if(tempParent->parent == NULL){
            node->neighbor[6].push_back(NULL);
            return;
        }
        else if(tempParent->parent != NULL){
            tempParent = tempParent->parent;
        }
    }
    while(path.size()){
        if(tempParent->nw == NULL){
            break;
        }
        else{
            if(path.back() == "ne"){
                tempParent = tempParent->nw;
                path.pop_back();
            }
            else if(path.back() == "se"){
                tempParent = tempParent->sw;
                path.pop_back();
            }
        }
    }
    eastTraverse(tempParent, node->neighbor);
}

void westTraverse(struct quadNode *node, vector<struct quadNode *> *neighbor){
    if(node->nw != NULL){//any child can be checked to find if child exists
        westTraverse(node->nw, neighbor);
        westTraverse(node->sw, neighbor);
    }
    else
        neighbor[2].push_back(node);
}

void westNeighbor(struct quadNode *node){
    vector<string> path;
    struct quadNode *tempNode = node;
    struct quadNode *tempParent;
    if(tempNode->parent == NULL){
        node->neighbor[2].push_back(NULL);
        return;
    }
    else if(tempNode->parent != NULL){
        tempParent = tempNode->parent;
    }
    while(tempNode == tempParent->nw || tempNode == tempParent->sw ){
        if(tempNode == tempParent->nw)
            path.push_back("nw");
        else if(tempNode == tempParent->sw)
            path.push_back("sw");
        tempNode = tempParent;
        if(tempParent->parent == NULL){
            node->neighbor[2].push_back(NULL);
            return;
        }
        else if(tempParent->parent != NULL){
            tempParent = tempParent->parent;
        }
    }
    while(path.size()){
        if(tempParent->nw == NULL){
            break;
        }
        else{
            if(path.back() == "nw"){
                tempParent = tempParent->ne;
                path.pop_back();
            }
            else if(path.back() == "sw"){
                tempParent = tempParent->se;
                path.pop_back();
            }
        }
    }
    westTraverse(tempParent, node->neighbor);
}

void northTraverse(struct quadNode *node, vector<struct quadNode *> *neighbor){
    if(node->ne != NULL){//any child can be checked to find if child exists
        northTraverse(node->ne, neighbor);
        northTraverse(node->nw, neighbor);
    }
    else
        neighbor[0].push_back(node);
}

void northNeighbor(struct quadNode *node){
    vector<string> path;
    struct quadNode *tempNode = node;
    struct quadNode *tempParent;
    if(tempNode->parent == NULL){
        node->neighbor[0].push_back(NULL);
        return;
    }
    else if(tempNode->parent != NULL){
        tempParent = tempNode->parent;
    }
    while(tempNode == tempParent->ne || tempNode == tempParent->nw ){
        if(tempNode == tempParent->ne)
            path.push_back("ne");
        else if(tempNode == tempParent->nw)
            path.push_back("nw");
        tempNode = tempParent;
        if(tempParent->parent == NULL){
            node->neighbor[0].push_back(NULL);
            return;
        }
        else if(tempParent->parent != NULL){
            tempParent = tempParent->parent;
        }
    }
    while(path.size()){
        if(tempParent->nw == NULL){//any node will do. just to check if child exists
            break;
        }
        else{
            if(path.back() == "ne"){
                tempParent = tempParent->se;
                path.pop_back();
            }
            else if(path.back() == "nw"){
                tempParent = tempParent->sw;
                path.pop_back();
            }
        }
    }
    northTraverse(tempParent, node->neighbor);
}

void southTraverse(struct quadNode *node, vector<struct quadNode *> *neighbor){
    if(node->ne != NULL){//any child can be checked to find if child exists
        southTraverse(node->se, neighbor);
        southTraverse(node->sw, neighbor);
    }
    else
        neighbor[4].push_back(node);
}

void southNeighbor(struct quadNode *node){
    vector<string> path;
    struct quadNode *tempNode = node;
    struct quadNode *tempParent;
    if(tempNode->parent == NULL){
        node->neighbor[4].push_back(NULL);
        return;
    }
    else if(tempNode->parent != NULL){
        tempParent = tempNode->parent;
    }
    while(tempNode == tempParent->se || tempNode == tempParent->sw ){
        if(tempNode == tempParent->se)
            path.push_back("se");
        else if(tempNode == tempParent->sw)
            path.push_back("sw");
        tempNode = tempParent;
        if(tempParent->parent == NULL){
            node->neighbor[4].push_back(NULL);
            return;
        }
        else if(tempParent->parent != NULL){
            tempParent = tempParent->parent;
        }
    }
    while(path.size()){
        if(tempParent->nw == NULL){//any node will do. just to check if child exists
            break;
        }
        else{
            if(path.back() == "se"){
                tempParent = tempParent->ne;
                path.pop_back();
            }
            else if(path.back() == "sw"){
                tempParent = tempParent->nw;
                path.pop_back();
            }
        }
    }
    southTraverse(tempParent, node->neighbor);
}

void nwTraverse(struct quadNode *node, vector<struct quadNode *> *neighbor){
    if(node->nw != NULL){//any child can be checked to find if child exists
        nwTraverse(node->nw, neighbor);
    }
    else
        neighbor[1].push_back(node);
}

void nwNeighbor(struct quadNode *node){
    vector<string> path;
    struct quadNode *tempNode = node;
    struct quadNode *tempParent;
    if(tempNode->parent == NULL){
        node->neighbor[1].push_back(NULL);
        return;
    }
    else if(tempNode->parent != NULL){
        tempParent = tempNode->parent;
    }
    while(tempNode == tempParent->nw || tempNode == tempParent->ne || tempNode == tempParent->sw){
        if(tempNode == tempParent->nw)
            path.push_back("nw");
        else if(tempNode == tempParent->ne)
            path.push_back("ne");
        else if(tempNode == tempParent->sw)
            path.push_back("sw");
        tempNode = tempParent;
        if(tempParent->parent == NULL){
            node->neighbor[1].push_back(NULL);
            return;
        }
        else if(tempParent->parent != NULL){
            tempParent = tempParent->parent;
        }
    }
    while(path.size()){
        if(tempParent->nw == NULL){//any node will do. just to check if child exists
            break;
        }
        else{
            if(path.back() == "nw"){
                tempParent = tempParent->se;
                path.pop_back();
            }
            else if(path.back() == "ne"){
                tempParent = tempParent->sw;
                path.pop_back();
            }
            else if(path.back() == "sw"){
                tempParent = tempParent->ne;
                path.pop_back();
            }
        }
    }
    nwTraverse(tempParent, node->neighbor);
}

void swTraverse(struct quadNode *node, vector<struct quadNode *> *neighbor){
    if(node->nw != NULL){//any child can be checked to find if child exists
        swTraverse(node->sw, neighbor);
    }
    else
        neighbor[3].push_back(node);
}

void swNeighbor(struct quadNode *node){
    vector<string> path;
    struct quadNode *tempNode = node;
    struct quadNode *tempParent;
    if(tempNode->parent == NULL){
        node->neighbor[3].push_back(NULL);
        return;
    }
    else if(tempNode->parent != NULL){
        tempParent = tempNode->parent;
    }
    while(tempNode == tempParent->nw || tempNode == tempParent->se || tempNode == tempParent->sw){
        if(tempNode == tempParent->nw)
            path.push_back("nw");
        else if(tempNode == tempParent->se)
            path.push_back("se");
        else if(tempNode == tempParent->sw)
            path.push_back("sw");
        tempNode = tempParent;
        if(tempParent->parent == NULL){
            node->neighbor[3].push_back(NULL);
            return;
        }
        else if(tempParent->parent != NULL){
            tempParent = tempParent->parent;
        }
    }
    while(path.size()){
        if(tempParent->nw == NULL){//any node will do. just to check if child exists
            break;
        }
        else{
            if(path.back() == "nw"){
                tempParent = tempParent->se;
                path.pop_back();
            }
            else if(path.back() == "se"){
                tempParent = tempParent->nw;
                path.pop_back();
            }
            else if(path.back() == "sw"){
                tempParent = tempParent->ne;
                path.pop_back();
            }
        }
    }
    swTraverse(tempParent, node->neighbor);
}

void neTraverse(struct quadNode *node, vector<struct quadNode *> *neighbor){
    if(node->nw != NULL){//any child can be checked to find if child exists
        swTraverse(node->ne, neighbor);
    }
    else
        neighbor[7].push_back(node);
}

void neNeighbor(struct quadNode *node){
    vector<string> path;
    struct quadNode *tempNode = node;
    struct quadNode *tempParent;
    if(tempNode->parent == NULL){
        node->neighbor[7].push_back(NULL);
        return;
    }
    else if(tempNode->parent != NULL){
        tempParent = tempNode->parent;
    }
    while(tempNode == tempParent->nw || tempNode == tempParent->se || tempNode == tempParent->ne){
        if(tempNode == tempParent->nw)
            path.push_back("nw");
        else if(tempNode == tempParent->se)
            path.push_back("se");
        else if(tempNode == tempParent->ne)
            path.push_back("ne");
        tempNode = tempParent;
        if(tempParent->parent == NULL){
            node->neighbor[7].push_back(NULL);
            return;
        }
        else if(tempParent->parent != NULL){
            tempParent = tempParent->parent;
        }
    }
    while(path.size()){
        if(tempParent->nw == NULL){//any node will do. just to check if child exists
            break;
        }
        else{
            if(path.back() == "nw"){
                tempParent = tempParent->se;
                path.pop_back();
            }
            else if(path.back() == "se"){
                tempParent = tempParent->nw;
                path.pop_back();
            }
            else if(path.back() == "sw"){
                tempParent = tempParent->ne;
                path.pop_back();
            }
        }
    }
    neTraverse(tempParent, node->neighbor);
}

void seTraverse(struct quadNode *node, vector<struct quadNode *> *neighbor){
    if(node->nw != NULL){//any child can be checked to find if child exists
        swTraverse(node->se, neighbor);
    }
    else
        neighbor[5].push_back(node);
}

void seNeighbor(struct quadNode *node){
    vector<string> path;
    struct quadNode *tempNode = node;
    struct quadNode *tempParent;
    if(tempNode->parent == NULL){
        node->neighbor[5].push_back(NULL);
        return;
    }
    else if(tempNode->parent != NULL){
        tempParent = tempNode->parent;
    }
    while(tempNode == tempParent->ne || tempNode == tempParent->se || tempNode == tempParent->sw){
        if(tempNode == tempParent->ne)
            path.push_back("ne");
        else if(tempNode == tempParent->se)
            path.push_back("se");
        else if(tempNode == tempParent->sw)
            path.push_back("sw");
        tempNode = tempParent;
        if(tempParent->parent == NULL){
            node->neighbor[5].push_back(NULL);
            return;
        }
        else if(tempParent->parent != NULL){
            tempParent = tempParent->parent;
        }
    }
    while(path.size()){
        if(tempParent->nw == NULL){//any node will do. just to check if child exists
            break;
        }
        else{
            if(path.back() == "ne"){
                tempParent = tempParent->sw;
                path.pop_back();
            }
            else if(path.back() == "se"){
                tempParent = tempParent->nw;
                path.pop_back();
            }
            else if(path.back() == "sw"){
                tempParent = tempParent->ne;
                path.pop_back();
            }
        }
    }
    seTraverse(tempParent, node->neighbor);
}

void findNeighbors(struct quadNode *node){
    eastNeighbor(node);
    westNeighbor(node);
    northNeighbor(node);
    southNeighbor(node);
    // neNeighbor(node);
    // nwNeighbor(node);
    // seNeighbor(node);
    // swNeighbor(node);
    return;    
}

void fillNewQuadrant(struct quadNode *parent){
        float x,y;
        x = (float)(parent->nw->x1 + parent->nw->x2)/2;
        y = (float)(parent->nw->y1 + parent->nw->y2)/2;
        parent->nw->boundaryXcord.push_back(x);
        parent->nw->boundaryYcord.push_back(y);

        x = (float)(parent->ne->x1 + parent->ne->x2)/2;
        y = (float)(parent->ne->y1 + parent->ne->y2)/2;
        parent->ne->boundaryXcord.push_back(x);
        parent->ne->boundaryYcord.push_back(y);

        x = (float)(parent->sw->x1 + parent->sw->x2)/2;
        y = (float)(parent->sw->y1 + parent->sw->y2)/2;
        parent->sw->boundaryXcord.push_back(x);
        parent->sw->boundaryYcord.push_back(y);

        x = (float)(parent->se->x1 + parent->se->x2)/2;
        y = (float)(parent->se->y1 + parent->se->y2)/2;
        parent->se->boundaryXcord.push_back(x);
        parent->se->boundaryYcord.push_back(y);

        return;
}

void fillEmptyLeafNodes(struct quadNode *leaf){
     float x,y;
     x = (float)(leaf->x1 + leaf->x2)/2;
     y = (float)(leaf->y1 + leaf->y2)/2;
     leaf->boundaryXcord.push_back(x);
     leaf->boundaryYcord.push_back(y);
}


void balanceTree(struct quadNode *root, int level){
    if(root==NULL)
        return;
    
    if(level==1){
        if(root->boundaryXcord.size()==0){
            fillEmptyLeafNodes(root);
        }

        findNeighbors(root);

        for(int i=0; i<8 ; i+=2){
            if(root->neighbor[i][0]){
                for(int j=0; j<root->neighbor[i].size(); j++){
                    if(root->level < root->neighbor[i][j]->level-1){
                        newLevel(root, root->x1, root->y1, root->x2, root->y2);
                        //fillNewQuadrant(root);
                        break;
                        
                    }
                }
            }
        }
    }

    else if(level>1){
        balanceTree(root->sw, level-1);//
        balanceTree(root->nw, level-1);//  
        balanceTree(root->ne, level-1);//
        balanceTree(root->se, level-1);//    
    }
    
}   

void bfsTraverse(struct quadNode *root){
    for(int i=0; i<=maxLevel; i++){
        balanceTree(root, i);
    }
}

// int pnpoly(struct quadNode *root, float *vertx, float *verty, float testx, float testy){//to check if point inside polygon
int pnpoly(struct quadNode *root, vector<float> vertx, vector<float> verty, float testx, float testy){
    int nvert = root->boundaryXcord.size();
    // int i, j, c = 0;
    // for(i = 0, j = nvert-1; i < nvert; j = i++){
    //     if( ((verty[i]>testy) != (verty[j]>testy)) &&
    //         (testx < (float)((vertx[j]-vertx[i])*(testy-verty[i])) / (float)((verty[j]-verty[i])) + vertx[i]) )
    //             c = !c; 
    // }
    // return c;//c=1 means inside polygon
    /* Iterate through each line */
    float x1, x2;
    int crossings = 0;
    for ( int i = 0; i < nvert; i++ ){

            /* This is done to ensure that we get the same result when
               the line goes from left to right and right to left */
            if ( vertx[i] < vertx[ (i+1)%nvert ] ){
                    x1 = vertx[i];
                    x2 = vertx[(i+1)%nvert];
            } else {
                    x1 = vertx[(i+1)%nvert];
                    x2 = vertx[i];
            }
        
            /* First check if the ray is possible to cross the line */
            if ( testx >= x1 && testx <= x2 && ( testy <= verty[i] || testy <= verty[(i+1)%nvert] ) ) {
                    //cout << testx << "\t" << testy << endl;
                    static const float eps = 0.001;
                
                    /* Calculate the equation of the line */
                    float dx = vertx[(i+1)%nvert] - vertx[i];
                    float dy = verty[(i+1)%nvert] - verty[i];
                    float k;
                
                    if ( fabs(dx) < eps ){
                            k = INFINITY;   // math.h
                    } else {
                            k = (float)dy/(float)dx;
                    }
                
                    float m = verty[i] - k * vertx[i];
                
                    /* Find if the ray crosses the line */
                    float y2 = k * testx + m;
                    if ( testy <= y2 ){
                            crossings++;
                    }
            }
    }
    
    if(crossings%2)
        return 1;
    else 
        return 0;
}

ofstream outfile("output.txt");//stores points without blanking

void postorder(struct quadNode *p, int indent=0){
    if(p != NULL){
        if(p->nw) postorder(p->nw, indent+4);
        if(p->ne) postorder(p->ne, indent+4);
        if(p->sw) postorder(p->sw, indent+4);
        if(p->se) postorder(p->se, indent+4);
        if (indent){
             std::cout << std::setw(indent) << ' ';
        }
        if(p->boundaryXcord.size()==1){
            cout<<"("<< p->boundaryXcord[0] << ",";
            cout<<      p->boundaryYcord[0] <<")"<< "\n ";
            outfile << p->boundaryXcord[0] <<"\t"<< p->boundaryYcord[0] << endl;
        }
        else
            cout<< p->boundaryXcord.size() << "\n ";
    }
}

void blanking(struct quadNode *root){
    float x, y;
    ifstream infile1("output.txt");
    ofstream outfile1("output1.txt");
    
    while(!infile1.eof()){
        infile1 >> x >> y;
        if(!pnpoly(root, root->boundaryXcord, root->boundaryYcord, x, y)){
            outfile1 << x <<"\t"<< y << endl;
            //cout << x << "\t" << y <<  endl;
        }
    }
}

int main(){
    struct quadNode *root = new struct quadNode;
    root->nw = root->ne = root->sw = root->se = NULL;
    root->level = 0;
    root->parent = NULL;
    // root->x1 = -200;//outer boundary coordinates
    // root->y1 = -200;//
    // root->x2 = 300; //
    // root->y2 = 300; //
    root->x1 = -6;
    root->y1 = -6;
    root->x2 = 7;
    root->y2 = 7;
    maxLevel = 0;

    ifstream infile("airfoil-points.txt");
    // ifstream infile("boundary.txt");
    int n;
    float x,y;
    int tmp1, tmp2;
    infile >> n;
    for(int i=0; i<n; i++){
        infile >> x >> y >> tmp1 >> tmp2;
        root->boundaryXcord.push_back(x);
        root->boundaryYcord.push_back(y); 
    }
    
    constructTree(root);
    bfsTraverse(root);
    //balanceTree(root);
    //fillNewQuadrant(root);
    postorder(root, 4);
    blanking(root);
    cout << endl << "MaxLevel: "<<maxLevel << endl;

    return 0;
}