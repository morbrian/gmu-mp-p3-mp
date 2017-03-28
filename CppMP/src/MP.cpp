#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>

#include <vector>
using std::vector;

#include <iostream>
using std::cout;

#include <cmath>
#include <cfloat>

//
// point_distance
// Calculate distance between 2 points (x1, y1) and (x2, y2)
// @return distance between points
//
double point_distance(const double x1, const double y1, const double x2, const double y2);

//
// Create random state (sto) from the goal region with probability = 0.1,
// or from the entire bounding box with probability 1 - p (using SampleState).
//
void select_sto(Simulator& m_simulator, double sto[]);

//
// dot product of 2D vectors a and b
// @param a array of 2 double
// @param b array of 2 double
//
inline double dotprod(const Point a, const Point b);

//
// return true if the point specified by v[] and t1
// are on the same side of the line t2, t3
// Used by Rectangle.
//
inline bool sameSide(double v[], Point t1, Point t2, Point t3);

MotionPlanner::MotionPlanner(Simulator * const simulator)
{
    m_simulator = simulator;
    m_totalWeight = 1.0;
    m_cacheWeight = false;
    
    Vertex *vinit = new Vertex();
    
    vinit->m_weight = 1.0; // 0 children means weight = 1
    vinit->m_parent   = -1;   
    vinit->m_nchildren= 0;    
    vinit->m_state[0] = m_simulator->GetRobotCenterX();
    vinit->m_state[1] = m_simulator->GetRobotCenterY();
    
    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;
    
}

MotionPlanner::~MotionPlanner(void)
{
    //do not delete m_simulator  
    
    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
        delete m_vertices[i];
}

void MotionPlanner::ExtendTree(const int    vid, 
                               const double sto[])
{
    //your code
    Vertex* start =  m_vertices.at(vid);
    
    double mag = point_distance(sto[0], sto[1], start->m_state[0], start->m_state[1]);
    
    // velocity vector from vid to sto
    double unit[] = {
        (sto[0] - start->m_state[0]) / mag,
        (sto[1] - start->m_state[1]) / mag
    };
    double steps = ceil(mag / m_simulator->GetDistOneStep());
    Vertex* new_vert;
    double cfg[] = { start->m_state[0], start->m_state[1] };
    int parent = m_vertices.size() - 1;
    for (double count = 0; count < steps; ++count) 
    {
        if (count == steps - 1)
        {
            // add the actual sto point on the last iteration,
            // which may be closer than distOneStep
            cfg[0] = sto[0];
            cfg[1] = sto[1];
        }
        else 
        {
            // else step closer to sto by a distance of OneStep.
            cfg[0] += unit[0] * m_simulator->GetDistOneStep();
            cfg[1] += unit[1] * m_simulator->GetDistOneStep();
        }
        
        m_simulator->SetRobotState(cfg);
        if (m_simulator->IsValidState()) 
        {
            new_vert = new Vertex();
            if (count == 0)
                new_vert->m_parent = vid;
            else 
                new_vert->m_parent = ++parent;
            new_vert->m_nchildren = 0;
            new_vert->m_state[0] = cfg[0];
            new_vert->m_state[1] = cfg[1];
            if (m_simulator->HasRobotReachedGoal())
            {
                new_vert->m_type = Vertex::TYPE_GOAL;
                AddVertex(new_vert);
                return;
            }
            AddVertex(new_vert);
        }
        else 
            break;
    }
}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);
    // given above
    //your code
    double sto[2];
    select_sto(*m_simulator, sto);
    
    int vid = rand() % m_vertices.size();
    
    ExtendTree(vid, sto);
    
    // given below
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
    // given above
    //your code
    double sto[2];
    select_sto(*m_simulator, sto);
    
    int vid = 0;
    double min_dist = FLT_MAX;
    double test_dist;
    vector<Vertex*>::iterator v = m_vertices.begin();
    vector<Vertex*>::iterator v_end = m_vertices.end();
    for (int id = 0; v != v_end; ++v, ++id)
    {
        test_dist = point_distance(sto[0], sto[1], (*v)->m_state[0], (*v)->m_state[1]);
        if (test_dist < min_dist)
        {
            min_dist = test_dist;
            vid = id;
        }
    }
    
    ExtendTree(vid, sto);
    
    // given below
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);
    //given above
    //your code
    double sto[2];
    select_sto(*m_simulator, sto);
    
    // indicate AddVertex should track individual and total weights
    m_cacheWeight = true;
    
    int vid = 0;    
    double r = double(rand()) / double(RAND_MAX) * m_totalWeight;
    double tw = 0;
    int n = m_vertices.size();
    vector<Vertex*>::iterator v = m_vertices.begin();
    vector<Vertex*>::iterator v_end = m_vertices.end();
    for (vid = 0; vid < n; ++vid, ++v)
    {
        tw += (*v)->m_weight;
        if (tw >= r)
            break;
    }

    ExtendTree(vid, sto);
    
    // given below
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendMyApproach(void)
{
    Clock clk;
    StartTime(&clk);
    // given above

    
    double sto[2];
    
    select_sto(*m_simulator, sto);
    
    // define the bounding rectangles around sto coordinates
    m_recth.defineHBounds(sto, *m_simulator);
    m_rectv.defineVBounds(sto, *m_simulator);
    
    // Get the preferred strategy for the random state
    Strategy s = (float(rand()) / float(RAND_MAX) > 0.5 ? HORIZONTAL : VERTICAL);
    
    
    int vid_rrt = 0;
    int vid_my = -1;
    double min_rrt = FLT_MAX;
    double min_my = FLT_MAX;
    double test_dist;
    vector<Vertex*>::iterator v = m_vertices.begin();
    vector<Vertex*>::iterator v_end = m_vertices.end();
    for (int id = 0; v != v_end; ++v, ++id)
    {
        test_dist = point_distance(sto[0], sto[1], (*v)->m_state[0], (*v)->m_state[1]);
        if (test_dist < min_rrt)
        {
            min_rrt = test_dist;
            vid_rrt = id;
        }
        
        if (s == HORIZONTAL)
        {
            if ((test_dist < min_my)
                && m_recth.inBounds(*v))
            {
                min_my = test_dist;
                vid_my = id;
            }
        }
        else if (s == VERTICAL)
        {
            if ((test_dist < min_my)
                && m_rectv.inBounds(*v))
            {
                min_my = test_dist;
                vid_my = id;
            }
        }
    }
    
    int vid = (vid_my >= 0 ? vid_my : vid_rrt);
    ExtendTree(vid, sto);

    // given below
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::AddVertex(Vertex * const v)
{
    if (m_cacheWeight)
    {
        v->m_weight = 1.0;
        m_totalWeight += 1.0;
    }
    
    if(v->m_type == Vertex::TYPE_GOAL)
        m_vidAtGoal = m_vertices.size();
    m_vertices.push_back(v); 
    if(v->m_parent >= 0 && !m_cacheWeight)
        (++m_vertices[v->m_parent]->m_nchildren);
    else if(v->m_parent >= 0)
    {
        Vertex* parent = m_vertices[v->m_parent];
        m_totalWeight -= parent->m_weight;
        parent->m_nchildren += 1;
        parent->m_weight = 1. / double(1 + parent->m_nchildren * parent->m_nchildren);
        m_totalWeight += parent->m_weight;
    }
}

void MotionPlanner::GetPathFromInitToGoal(std::vector<int> *path) const
{
    std::vector<int> rpath;
    
    rpath.clear();
    
    int i = m_vidAtGoal;
    do
    {
        rpath.push_back(i);
        i = m_vertices[i]->m_parent;	
    } 
    while(i >= 0);
    
    path->clear();
    for(int i = rpath.size() - 1; i >= 0; --i)
        path->push_back(rpath[i]);
}

// distance from starting point (x1,y1) to end point (x2, y2)
double point_distance(const double x1, const double y1, const double x2, const double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// create a random state 0.1 near goal, 1 - 0.1 in entire bb.
void select_sto(Simulator& m_simulator, double sto[])
{
    double g_radius = m_simulator.GetGoalRadius();

    float chance = float(rand() % 10);
    
    // 9 out of 10 times, choose from the whole box
    if (chance > 0)
    {
        // select from bounding box
        m_simulator.SampleState(sto);
    }
    else 
    {
        // select from goal region (within radius distance of center point)
        double r = double(rand());
        double r_part = (double)(rand() % 1000) / 1000;
        sto[0] = m_simulator.GetGoalCenterX() + cos(r) * g_radius * r_part * 
                        (rand() % 2 == 1 ? 1 : -1);
        sto[1] = m_simulator.GetGoalCenterY() + sin(r) * g_radius * r_part * 
                        (rand() % 2 == 1 ? 1 : -1);
    }
}


bool sameSide(const double v[], const Point& p, const Point& t1, const Point& t2)
{
    // t2 - t1
    Point vec_side = 
    {
        t2.m_x - t1.m_x,
        t2.m_y - t1.m_y
    };
    
    // perpindicular to vec_side
    Point vec_perp = 
    {
        - vec_side.m_y,
        vec_side.m_x
    };
    
    // v - t1
    Point vec_v =
    {
        v[0] - t1.m_x,
        v[1] - t1.m_y
    };
    
    // p - t1
    Point vec_p =
    {
        p.m_x - t1.m_x,
        p.m_y - t1.m_y
    };
    
    double dp_p = dotprod(vec_perp, vec_p);
    double dp_v = dotprod(vec_perp, vec_v);
    
    return (dp_p >= 0 && dp_v >= 0) || (dp_p < 0 && dp_v < 0);
    
}

void 
Rectangle::defineHBounds(const double *sto, Simulator& m_simulator)
{
    const double* bbox = m_simulator.GetBoundingBox();
    // horizontal rectangle
    vertices[0].m_x = bbox[0];
    vertices[0].m_y = sto[1] + (1.0 * m_simulator.GetRobotRadius());
    vertices[1].m_x = bbox[0];
    vertices[1].m_y = sto[1] - (1.0 * m_simulator.GetRobotRadius());
    vertices[2].m_x = bbox[2];
    vertices[2].m_y = sto[1] - (1.0 * m_simulator.GetRobotRadius());
    vertices[3].m_x = bbox[2];
    vertices[3].m_y = sto[1] + (1.0 * m_simulator.GetRobotRadius());    
}

void 
Rectangle::defineVBounds(const double *sto, Simulator& m_simulator)
{
    const double* bbox = m_simulator.GetBoundingBox();
    // horizontal rectangle
    vertices[0].m_x = sto[0] + (1.0 * m_simulator.GetRobotRadius());
    vertices[0].m_y = bbox[1];
    vertices[1].m_x = sto[0] - (1.0 * m_simulator.GetRobotRadius());
    vertices[1].m_y = bbox[1];
    vertices[2].m_x = sto[0] - (1.0 * m_simulator.GetRobotRadius());
    vertices[2].m_y = bbox[3];
    vertices[3].m_x = sto[0] + (1.0 * m_simulator.GetRobotRadius());    
    vertices[3].m_y = bbox[3];
    
}

bool 
Rectangle::inBounds(const Vertex* v)
{
    return sameSide(v->m_state, vertices[0], vertices[1], vertices[2])
    && sameSide(v->m_state, vertices[1], vertices[2], vertices[3])
    && sameSide(v->m_state, vertices[2], vertices[3], vertices[0])
    && sameSide(v->m_state, vertices[3], vertices[0], vertices[1]);
}

// dot product of 2 vectors
inline double dotprod(const Point a, const Point b)
{
    return (a.m_x * b.m_x + a.m_y * b.m_y);
}



