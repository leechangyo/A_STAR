// created by chan 2021-03-30
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

// input
const string M50_1[]={
    "/Users/chan/Documents/project/A_star/assets/Map50_1.bmp",
    "/Users/chan/Documents/project/A_star/assets/Map50_1_Out_2.png",

};

// constant value
const int NODE_TYPE_ZERO = 0;
const int NODE_TYPE_OBSTACLE = 1;
const int NODE_TYPE_START = 2;
const int NODE_TYPE_END = 3;
const int NODE_FLAGE_UNDEFINED = 0;
const int NODE_FLAGE_CLOSED = -1;
const int ALLOW_VERTEX_PASSTHROUGH = 0;
const int NODE_FLAG_OPEN = 1;

const int G_DIRECT = 10;
const int G_SKEW = 14;




// functions
class MapSize{
    public:
    // variable
    unsigned long width = 0;
    unsigned long height = 0;
    unsigned long size = 0;

    // initizlied 
    MapSize() {}

    MapSize(unsigned long width, unsigned long height) {
        this->width = width;
        this->height = height;
        this->size = width * height;
    }

};
class MapNode{
    public:
    // 휴리스틱에 필요한 변수
    // g = the movement cost to move from the starting point to a given square on the grid, following the path generated to get there.
    // h = the estimated movement cost to move from that given square on the grid to the final destination.
    int x = -1;
    int y = -1;
    int h = 0;
    int g = 0;
    int type = NODE_TYPE_ZERO;
    int flage = NODE_FLAGE_UNDEFINED;
    MapNode *parent = 0;

    // Initilized
    MapNode() {}

    MapNode(int x, int y, int type = NODE_TYPE_ZERO, int flag = NODE_FLAGE_UNDEFINED, MapNode* parent = 0)
    {
        this->x = x;
        this->y = y;
        this->type = type;
        this->flage = flag;
        this->parent = parent;
    }

    int f()
    {
        return g + h;
    }

};

// global variable
Mat map;
const string *FILE_PATH = M50_1;
MapSize mapSize;
vector<MapNode> mapData;
MapNode* startnode;
MapNode* endnode;
MapNode* mapAt(int x, int y);
vector<MapNode *> openList;


vector<MapNode*> neighbors(MapNode* node);

/* the standard heuristic is the manhattan distance 
look at your cost functions and see what the least cost is 
for moving from one space to another 
the heuristic should be cost times manhattan distance*/

inline int manhattan_distance(MapNode* node1, MapNode* node2)
{
    return abs(node2->x - node1->x) + abs(node2->y - node1->y);
}

inline int diagonal_distance(MapNode* node1, MapNode* node2)
{
    return max(abs(node2->x - node1->x), abs(node2->y - node1->y));
}

int computeH(MapNode* node1, MapNode* node2)
{
    // return abs(node1->x - node2->x) + abs(node1->y - node2->y);
    if(ALLOW_VERTEX_PASSTHROUGH)
    {
        // euclidian distance
        return diagonal_distance(node1,node2)*G_SKEW;
    }
    else
    {
        // manhatan
        return manhattan_distance(node1,node2)*G_DIRECT;
    }
}

int computeG(MapNode* node1, MapNode* node2)
{
    int dX = abs(node1->x - node2->x);
    int dY = abs(node1->y - node2->y);
    if(dX > dY)
    {
        return 14 * dY + 10 * (dX-dY);
    }
    else
    {
        return 14 * dX + 10 * (dY - dX);
    }
}

MapNode* mapAt(int x, int y)
{
    if(x<0 || y<0 || x >= mapSize.width || y >= mapSize.height) return 0;
    return &mapData[y * mapSize.width + x];
};

void drawOpenList(){
    for(int i = 0; i<openList.size(); i++){
        MapNode *node = openList[i];
        if(node == startnode || node == endnode) continue;
        map.at<Vec3b>(node->y, node->x) = Vec3b(210,210,210);
    }
}

vector<MapNode *> find() {
    vector<MapNode *> path;
    cout << "Finding started!" << endl;
    int iteration = 0;
    MapNode *node;
    MapNode *reversedPtr = 0;
    while (openList.size() > 0) {
        node = openList.at(0);
            //오픈 리스트 노드의 합(처음부터 현재까지 데스티네이션 고려 ) 크기가 첫번째 노드 보다 작고 오픈리스트의 무브먼트 가중치가 노드보다 작을 경우 
             // 최소 가중치가 낮은 합을 찾고 pop off(즉 close를 한다) 
        for (int i = 0, max = openList.size(); i < max; i++) {
            if (openList[i]->f() <= node->f() && openList[i]->h < node->h) {
                node = openList[i];
            }
        }
       // 가장 낮은 가중치의 값을 openlist에서 없애고 클로즈를 한다.
        openList.erase(remove(openList.begin(), openList.end(), node), openList.end());
        node->flage = NODE_FLAGE_CLOSED;
        cout << iteration++ << endl;
        cout << "   Current node " << node->x << ", " << node->y << " ..." << endl;
        if (node->parent != 0)
            cout << "       ... parent " << node->parent->x << ", " << node->parent->y << endl;
        if (node == endnode) {
            cout << "Reached the target node." << endl;
            reversedPtr = node;
            break;
        }
        // 옮겨진 노드 주변에 노드들을 탐색
        vector<MapNode *> neighborNodes = neighbors(node);
        cout << "       ... has " << neighborNodes.size() << " neighbors" << endl;
        for (int i = 0; i < neighborNodes.size(); i++) {
            MapNode *_node = neighborNodes[i];
            if (_node->flage == NODE_FLAGE_CLOSED || _node->type == NODE_TYPE_OBSTACLE) {
                continue;
            }
            int g = node->g + computeG(_node, node);
            // 현재 계산된 노드의 g값이 node_->g값 보다 작으면 업데이트, node_->g업데이트 
            if (_node->flage == NODE_FLAGE_UNDEFINED || g < _node->g) {
                _node->g = g;
                _node->h = computeH(_node, endnode);
                _node->parent = node;
                if (_node->flage != NODE_FLAG_OPEN) {
                    _node->flage = NODE_FLAG_OPEN;
                    openList.push_back(_node);
                }
            }
        }
        drawOpenList();
        if (openList.size() <= 0) break;

    }
    if (reversedPtr == 0) {
        cout << "Target node is unreachable." << endl;
    } else {
        MapNode *_node = reversedPtr;
        while (_node->parent != 0) {
            path.push_back(_node);
            _node = _node->parent;
        }
        reverse(path.begin(), path.end());
    }
    return path;
}

vector<MapNode*> neighbors(MapNode* node)
{
    vector<MapNode*> available;
    MapNode* node_;

    // L
    if((node_ = mapAt(node->x-1, node->y))!= 0) available.push_back(node_);
    // T
    if((node_ = mapAt(node->x, node->y-1))!= 0) available.push_back(node_);
    // R
    if((node_ = mapAt(node->x+1, node->y))!= 0) available.push_back(node_);
    // B
    if((node_ = mapAt(node->x, node->y+1))!= 0) available.push_back(node_);
    
    if(ALLOW_VERTEX_PASSTHROUGH)
    {
        // LT
        if((node_ = mapAt(node->x-1, node->y-1))!= 0) available.push_back(node_);
        // RT
        if((node_ = mapAt(node->x+1, node->y-1))!= 0) available.push_back(node_);
        // LB
        if((node_ = mapAt(node->x-1, node->y+1))!= 0) available.push_back(node_);
        // RB
        if((node_ = mapAt(node->x+1, node->y+1))!= 0) available.push_back(node_);
    }

    return available;
}

void drawPath(Mat &map, vector<MapNode*> path)
{
    cvtColor(map, map, COLOR_BGR2HSV);
    for(int i = 0; i<path.size() -1; i++)
    {
        MapNode* node= path[i];
        // give a colar
        map.at<Vec3b>(node->y, node->x) = Vec3b(20 + (1.0 - ((double)i)/path.size())*80,200,250);
        cout << "->(" << node->x << "," << node->y << ")";
    }
    cout << endl;
    cvtColor(map,map, COLOR_HSV2BGR);
    resize(map,map,Size(500,500),0,0, INTER_NEAREST);
}


int main(){
    map = imread(FILE_PATH[0]);
    Mat resized;
    resize(map, resized, Size(500,500), 0,0, INTER_NEAREST);
    imwrite(FILE_PATH[1], resized);

    mapSize = MapSize(map.cols, map.rows);
    mapData = vector<MapNode>(mapSize.size);
    cout << "MapSize(" << mapSize.width << ", " << mapSize.height << ", " << mapSize.size << ")" << endl;

    // reading image pixel and put in MapNode.
    for(int y = 0; y < map.rows; y++){
        for(int x = 0; x<map.cols; x++){
            // case 1 :if free space(white in image)
            if(map.at<Vec3b>(y,x) == Vec3b(255,255,255)){
                mapData[y*mapSize.width+x] = MapNode(x,y,NODE_TYPE_ZERO);
            }
            // case 2 : if obstacle(black in image)
            else if (map.at<Vec3b>(y,x) == Vec3b(0,0,0)){
                mapData[y*mapSize.width+x] = MapNode(x,y,NODE_TYPE_OBSTACLE);
            }
            // case 3 : start point ( blue in image)
            else if (map.at<Vec3b>(y,x) == Vec3b(255,0,0)){
                mapData[y*mapSize.width+x] = MapNode(x,y,NODE_TYPE_START);
                startnode = &mapData[y*mapSize.width+x];
            }
            // case 4 : end point (red in image)
            else if (map.at<Vec3b>(y,x) == Vec3b(0,0,255)){
                mapData[y*mapSize.width+x] = MapNode(x,y,NODE_TYPE_END);
                endnode = &mapData[y*mapSize.width+x];
            }
            // case 5 : rest of color in pixel set as obstacle.
            else{
                map.at<Vec3b>(y,x) = Vec3b(0,0,0);
                mapData[y*mapSize.width+x] = MapNode(x,y,NODE_TYPE_OBSTACLE);
            }
        }

    }

    for(int y = 0; y<mapSize.height; y++)
    {
        for(int x = 0; x<mapSize.width; x++)
        {
            cout << mapAt(x, y)->type << " ";
        }
        cout<<endl;
    }

    int i;
    cout << "startNoe = (" << startnode->x << ", " << startnode->y << ")"<<endl;
    cout << "TargetNoe = (" << endnode->x << ", " << endnode->y << ")"<<endl;
    cout<< ((i=1) == 1) << endl;

    openList.push_back(startnode);
    vector<MapNode *> path = find();

    drawPath(map,path);
    imwrite(FILE_PATH[1],map);

    return 0;
}