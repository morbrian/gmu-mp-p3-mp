#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#include "Simulator.hpp"

struct Vertex
{
    enum
	{
	    TYPE_NONE = 0,
	    TYPE_INIT = 1,
	    TYPE_GOAL = 2
	};
	
    int    m_parent;
    double m_state[Simulator::STATE_NR_DIMS];
    int    m_type;
    int    m_nchildren;
    double m_weight;
    
};

enum Strategy
{
    VERTICAL = 0,
    HORIZONTAL = 1,
    RRT = 2
};

struct Point
{
    double m_x;
    double m_y;
};

struct Rectangle
{
    Point vertices[4];
    void defineVBounds(const double *sto, Simulator& m_simulator);
    void defineHBounds(const double *sto, Simulator& m_simulator);
    bool inBounds(const Vertex* v);
};
    

class MotionPlanner
{
public:
    MotionPlanner(Simulator * const simulator);
            
    ~MotionPlanner(void);

    void ExtendRandom(void);

    void ExtendRRT(void);

    void ExtendEST(void);

    void ExtendMyApproach(void);
        
protected:
    bool IsProblemSolved(void)
    {
	return m_vidAtGoal >= 0;
    }

    void GetPathFromInitToGoal(std::vector<int> *path) const;

    void AddVertex(Vertex * const v);

    void ExtendTree(const int    vid,
		    const double sto[]);
    
    Simulator            *m_simulator;
    std::vector<Vertex *> m_vertices;
    int                   m_vidAtGoal;
    double                m_totalSolveTime;
    
    // my code added
    double m_totalWeight;
    bool m_cacheWeight;
    Rectangle m_rectv;
    Rectangle m_recth;
    
    friend class Graphics;    
};

#endif
