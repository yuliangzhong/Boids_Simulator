#ifndef BOIDS_H
#define BOIDS_H
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Sparse>
template <typename T, int dim>
using Vector = Eigen::Matrix<T, dim, 1, 0, dim, 1>;

template <typename T, int n, int m>
using Matrix = Eigen::Matrix<T, n, m, 0, n, m>;

// Define methods here
enum MethodTypes
{
    FREEFALL=0, CIRCULAR_MOTION=1, COHESION=2, ALIGNMENT=3, SEPARATION=4, COLLISION_AVOID=5, LEADER=6, CA_BEHAVE=7
};

template <class T, int dim>
class Boids
{
    typedef Matrix<T, Eigen::Dynamic, 1> VectorXT;
    typedef Matrix<T, dim,Eigen::Dynamic> TVStack;
    typedef Vector<T, dim> TV;
    typedef Matrix<T, dim, dim> TM;

private:
    TVStack positions;  // a matrix (dim * n)
    TVStack velocities; // a matrix (dim * n)
    int n;
    bool update = false;
    TV mouse_pos = TV(0,0);
    TVStack A_pos, A_vel;
    TVStack B_pos, B_vel;
    int cnt = 0;

    // params configuration here!---------------------------------------
    float h = 0.0005;                // the step size // speed of simulation
    int updateMode = 1;              // updateMode = 0/1/other int
    
    float cohesion_radius = 0.5;
    float repel_radius = 0.08;
    float ck = 10;                   // cohesion gain (pos drag)
    float ak = 1;                    // alignment gain (vel drag)
    float rk = 500;                  // seperation repel gain

    float obs_radius = 0.2;          // obstacle radius
    float eyesight_range = 1;        // if see obstacle, get repelled
    float obs_effect_band = 0.2;     // repelled if too close
    float ok = 10;                   // obstacle repel gain
    float obs_repel_power = 0.5;     // repel power x^?
    TV obs_pos = TV(0, 0);           // obstacle position

    TV fixed_goal_pos = TV(-1.5,-0.5);  // fixed goal position
    float max_drag = 20;                // goal attraction force maximum
    float gpk = 20;
    float gdk = 8;                      // goal attraction D-gain

    int breed_gap = 1000;
    float bound_edge = 0.1;
    float safe_edge = 1.75;
    float bound_repel_acc = 500;
    float breed_range = 0.09;           // slightly bigger than repel range
    float death_range = 0.15;
    float enemy_kill = 3;
    float repel_death_ratio = 0.6;
    float repel_num = 6;
    // ----------------------------------------------------------------

public:
    Boids() :n(1) {}
    Boids(int n) :n(n) {initializePositions();}
    ~Boids() {}

    void setParticleNumber(int n) {n = n;}
    int getParticleNumber() { return n; }

    void initializePositions(MethodTypes type = FREEFALL)
    {
        // Basic Spawn
        auto RAND = [&](T dummy) {return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);};
        TVStack bias = TVStack::Ones(dim,n);
        positions = TVStack::Zero(dim, n).unaryExpr(RAND)- 0.5*bias; //randomly spawn position in [-0.5,0.5]*[-0.5,0.5]
        velocities = TVStack::Zero(dim, n); // basic initial velocity is 0

        if(type == CIRCULAR_MOTION)
        {
            // for circular motion, the velocity is (-yi,xi)
            velocities.row(0) = -positions.row(1);
            velocities.row(1) = positions.row(0);
        }
        else if (type == COLLISION_AVOID)
        {
            positions = TVStack::Zero(dim, n).unaryExpr(RAND) + 0.5*bias; //randomly spawn position in [0.5,1.5]*[0.5,1.5]
        }
        else if (type == CA_BEHAVE)
        {
            A_pos = TVStack::Zero(dim, int(n/2)).unaryExpr(RAND) + 0.5*bias;
            B_pos = TVStack::Zero(dim, int(n/2)).unaryExpr(RAND) - 1.5*bias;
            A_vel = B_vel = TVStack::Zero(dim, int(n/2)).unaryExpr(RAND) - 0.5*bias;
        }
        else if(type != FREEFALL)
        {
            // for flocking behavior, randomly init velocities in [-0.5,0.5]*[-0.5,0.5]
            velocities = TVStack::Zero(dim, n).unaryExpr(RAND) - 0.5*bias;
        }
    }

//Xupdate: update new position given pos and vel
    TVStack Xupdate(TVStack pos, TVStack vel, bool if_half_h = false)
    {
        if(if_half_h) return pos + h/2*vel;
        else return pos + h*vel;
    }
//Vupdate: update new velocity given current velocity and acceleration
    TVStack Vupdate(TVStack vel, TVStack acc, bool if_half_h = false)
    {
        if(if_half_h) return vel + h/2*acc;
        else return vel + h*acc;
    }

// -----------------------------------------------------------------------------------
// getAcc: main function implement for this exercises
// compute acceleration for each particle, given currentMethod and pos
    TVStack getAcc(MethodTypes type, TVStack pos)
    {
        TVStack acc = TVStack::Zero(dim,n);
        if(type == FREEFALL)
        {
            acc.row(0) = TV::Zero();
            acc.row(1) = 9.81*TV::Ones();
            return acc;
        }
        else if (type == CIRCULAR_MOTION)
        {
            return -pos;
        }
        else if (type == COHESION)
        {
            for(int i=0;i<n;i++)
            {
                TV neighbor_pos_sum = TV::Zero();
                int neighbor_cnt = 0;
                for(int j=0;j<n;j++)
                {
                    if(j == i) continue;
                    if((pos.col(i)-pos.col(j)).norm()<=cohesion_radius) 
                    {
                        neighbor_pos_sum += pos.col(j);
                        neighbor_cnt ++;
                    }
                }
                if(neighbor_cnt != 0)
                {
                    neighbor_pos_sum /= neighbor_cnt;
                    acc.col(i) = ck * (neighbor_pos_sum - pos.col(i));
                }
            }
            return acc;
        }
        else if (type == ALIGNMENT)
        {
            TVStack vel = TVStack::Zero(dim,n);
            vel = getVelocities();

            for(int i=0;i<n;i++)
            {
                TV neighbor_pos_sum = TV::Zero();
                TV neighbor_vel_sum = TV::Zero();
                int neighbor_cnt = 0;
                for(int j=0;j<n;j++)
                {
                    if(j == i) continue;
                    if((pos.col(i)-pos.col(j)).norm()<=cohesion_radius) 
                    {
                        neighbor_pos_sum += pos.col(j);
                        neighbor_vel_sum += vel.col(j);
                        neighbor_cnt ++;
                    }
                }
                if(neighbor_cnt != 0)
                {
                    neighbor_pos_sum /= neighbor_cnt;
                    neighbor_vel_sum /= neighbor_cnt;
                    acc.col(i) = ck * (neighbor_pos_sum - pos.col(i)) + ak * (neighbor_vel_sum - vel.col(i));
                }
            }
            return acc;
        }
        else if (type == SEPARATION)
        {
            TVStack vel = TVStack::Zero(dim,n);
            vel = getVelocities();

            for(int i=0;i<n;i++)
            {
                TV neighbor_pos_sum = TV::Zero();
                TV neighbor_vel_sum = TV::Zero();
                TV neighbor_repel_sum = TV::Zero();

                int neighbor_cnt = 0;
                for(int j=0;j<n;j++)
                {
                    if(j == i) continue;
                    if((pos.col(i)-pos.col(j)).norm()<=cohesion_radius) 
                    {
                        neighbor_pos_sum += pos.col(j);
                        neighbor_vel_sum += vel.col(j);
                        neighbor_cnt ++;
                    }
                    if((pos.col(i)-pos.col(j)).norm()<=repel_radius) 
                    {
                        neighbor_repel_sum += rk*(pos.col(i)-pos.col(j));
                    }
                }
                if(neighbor_cnt != 0)
                {
                    neighbor_pos_sum /= neighbor_cnt;
                    neighbor_vel_sum /= neighbor_cnt;
                    acc.col(i) = ck * (neighbor_pos_sum - pos.col(i)) + ak * (neighbor_vel_sum - vel.col(i)) + neighbor_repel_sum;
                }
            }
            return acc;
        }
        else if (type == COLLISION_AVOID)
        {
            TVStack vel = TVStack::Zero(dim,n);
            vel = getVelocities();

            for(int i=0;i<n;i++)
            {
                TV neighbor_pos_sum = TV::Zero();
                TV neighbor_vel_sum = TV::Zero();
                TV neighbor_repel_sum = TV::Zero();

                int neighbor_cnt = 0;
                for(int j=0;j<n;j++)
                {
                    if(j == i) continue;
                    if((pos.col(i)-pos.col(j)).norm()<=cohesion_radius) 
                    {
                        neighbor_pos_sum += pos.col(j);
                        neighbor_vel_sum += vel.col(j);
                        neighbor_cnt ++;
                    }
                    if((pos.col(i)-pos.col(j)).norm()<=repel_radius) 
                    {
                        neighbor_repel_sum += rk*(pos.col(i)-pos.col(j));
                    }
                }
                if(neighbor_cnt != 0)
                {
                    neighbor_pos_sum /= neighbor_cnt;
                    neighbor_vel_sum /= neighbor_cnt;
                    acc.col(i) = ck * (neighbor_pos_sum - pos.col(i)) + ak * (neighbor_vel_sum - vel.col(i)) + neighbor_repel_sum;
                }
                if((pos.col(i)-obs_pos).norm() <= obs_radius + eyesight_range)
                {
                    float N = (pos.col(i)-obs_pos).norm();
                    float x = N -obs_radius;
                    if(x<obs_effect_band) acc.col(i) += ok*pow(x,-obs_repel_power)*(pos.col(i)-obs_pos).normalized();
                    acc.col(i) += ok*pow(obs_effect_band,-obs_repel_power)*(pos.col(i)-obs_pos)/N;
                }
                float drag = gpk*(fixed_goal_pos-pos.col(i)).norm();
                acc.col(i) += (drag > max_drag ? max_drag : drag)*(fixed_goal_pos-pos.col(i)).normalized();
                acc.col(i) += gdk*(-vel.col(i));
            }
            return acc;
        }
        else if (type == LEADER)
        {
            TVStack vel = TVStack::Zero(dim,n);
            vel = getVelocities();
            
            for(int i=1;i<n;i++)
            {
                TV neighbor_pos_sum = TV::Zero();
                TV neighbor_vel_sum = TV::Zero();
                TV neighbor_repel_sum = TV::Zero();

                int neighbor_cnt = 0;
                for(int j=0;j<n;j++)
                {
                    if(j == i) continue;
                    if((pos.col(i)-pos.col(j)).norm()<=cohesion_radius) 
                    {
                        neighbor_pos_sum += pos.col(j);
                        neighbor_vel_sum += vel.col(j);
                        neighbor_cnt ++;
                    }
                    if((pos.col(i)-pos.col(j)).norm()<=repel_radius) 
                    {
                        neighbor_repel_sum += rk*(pos.col(i)-pos.col(j));
                    }
                }
                if(neighbor_cnt != 0)
                {
                    neighbor_pos_sum /= neighbor_cnt;
                    neighbor_vel_sum /= neighbor_cnt;
                    acc.col(i) = ck * (neighbor_pos_sum - pos.col(i)) + 0.06*ak * (neighbor_vel_sum - vel.col(i)) + 0.5*neighbor_repel_sum;
                }
                float drag = gpk*(pos.col(0)-pos.col(i)).norm();
                acc.col(i) += (drag > max_drag ? max_drag : drag)*(pos.col(0)-pos.col(i)).normalized();
                acc.col(i) += 0.3*gdk*(vel.col(0)-vel.col(i));
            }
            float target_drag = gpk*(mouse_pos-pos.col(0)).norm();
            acc.col(0) += (target_drag > max_drag ? max_drag : target_drag)*(mouse_pos-pos.col(0)).normalized();
            acc.col(0) += 0.5*gdk*(-vel.col(0));
            return acc;
        }
    }
//---------------------------------------------------------------------------------------------------
    void breed(TVStack &pos, TVStack &vel)
    {
        int n = pos.cols(); // n should be fixed
        for(int i=0;i<n-1;i++)
        {
            for(int j=i+1;j<n;j++)
            {
                if((pos.col(i)-pos.col(j)).norm()<breed_range)
                {
                    TV child_pos = (pos.col(i) + pos.col(j))/2;
                    TV child_vel = (vel.col(i) + vel.col(j))/2;
                    pos.conservativeResize(pos.rows(), pos.cols()+1);
                    pos.col(pos.cols()-1) = child_pos;
                    vel.conservativeResize(vel.rows(), vel.cols()+1);
                    vel.col(vel.cols()-1) = child_vel;
                }
            }
        }
    }
    void removeCol(TVStack &Matrix, int i)
    {
        int rows = Matrix.rows();
        int cols = Matrix.cols();
        if(i>=0 && i<cols)
        {
            Matrix.block(0,i,rows,cols-i) = Matrix.rightCols(cols-i-1);
            Matrix.conservativeResize(rows,cols-1);
        }
    }
    void attack(TVStack &posA, TVStack &posB, TVStack &velA, TVStack &velB)
    {
        TVStack old_posA = posA;
        TVStack old_posB = posB;
        for(int i=0;i < old_posA.cols();i++)
        {
            int enemy_cnt = 0;
            int repel_cnt = 0;
            for(int j=0;j < old_posB.cols();j++)
            {
                if((old_posB.col(j)-old_posA.col(i)).norm()<death_range) enemy_cnt++;
            }
            for(int j=0;j < old_posA.cols();j++)
            {
                if((old_posA.col(j)-old_posA.col(i)).norm()<repel_death_ratio*repel_radius) repel_cnt++;
            }
            if(enemy_cnt>=enemy_kill||repel_cnt>=repel_num)
            {
                removeCol(posA,i);
                removeCol(velA,i);
            }
        }
        
        for(int i=0;i < old_posB.cols();i++)
        {
            int enemy_cnt = 0;
            int repel_cnt = 0;
            for(int j=0;j < old_posA.cols();j++)
            {
                if((old_posA.col(j)-old_posB.col(i)).norm()<death_range) enemy_cnt++;
            }
            for(int j=0;j < old_posB.cols();j++)
            {
                if((old_posB.col(j)-old_posB.col(i)).norm()<repel_death_ratio*repel_radius) repel_cnt++;
            }
            if(enemy_cnt>=enemy_kill||repel_cnt>=repel_num)
            {
                removeCol(posB,i);
                removeCol(velB,i);
            }
        }
    }
    TVStack CA_acc(TVStack pos, TVStack vel, bool isControlled = false)
    {
        TVStack acc = TVStack::Zero(dim,pos.cols());
        for(int i=0;i<pos.cols();i++)
        {
            TV neighbor_pos_sum = TV::Zero();
            TV neighbor_vel_sum = TV::Zero();
            TV neighbor_repel_sum = TV::Zero();

            int neighbor_cnt = 0;
            for(int j=0;j<pos.cols();j++)
            {
                if(j == i) continue;
                if((pos.col(i)-pos.col(j)).norm()<=cohesion_radius) 
                {
                    neighbor_pos_sum += pos.col(j);
                    neighbor_vel_sum += vel.col(j);
                    neighbor_cnt ++;
                }
                if((pos.col(i)-pos.col(j)).norm()<=repel_radius) 
                {
                    neighbor_repel_sum += rk*(pos.col(i)-pos.col(j));
                }
            }
            if(neighbor_cnt != 0)
            {
                neighbor_pos_sum /= neighbor_cnt;
                neighbor_vel_sum /= neighbor_cnt;
                acc.col(i) = ck * (neighbor_pos_sum - pos.col(i)) + ak * (neighbor_vel_sum - vel.col(i)) + neighbor_repel_sum;
            }
            if(pos.col(i)[0] > +safe_edge-bound_edge)  acc.col(i)[0]+= -bound_repel_acc;
            if(pos.col(i)[0] < -safe_edge+bound_edge)  acc.col(i)[0]+= +bound_repel_acc;
            if(pos.col(i)[1] > +safe_edge-bound_edge)  acc.col(i)[1]+= -bound_repel_acc;
            if(pos.col(i)[1] < -safe_edge+bound_edge)  acc.col(i)[1]+= +bound_repel_acc;

            if(isControlled)
            {
                // strategy 1
                //acc.col(i) += -pos.col(i);// quick attack, quick retreat
                // strategy 2
                /*
                TV my_pos_avg = pos.colwise().mean();
                TVStack Bpos = get_B_pos();
                TV nearest_enemy = TV(0,0);
                if(Bpos.cols()>0)
                {
                    nearest_enemy = Bpos.col(0);
                    for(int i=0;i<Bpos.cols();i++)
                    {
                        if(Bpos.col(i)[0]>nearest_enemy[0])
                        {
                            nearest_enemy = Bpos.col(i);
                        }
                    }
                    acc.col(i) += (nearest_enemy - pos.col(i));
                }
                */
                // strategy 3
                TVStack Bpos = get_B_pos();
                if(Bpos.cols()>0 && pos.cols()<180)
                {
                    TV nearest_enemy = TV(0,0);
                    TV avg = get_B_pos().colwise().mean();
                    TV my_pos_avg = pos.colwise().mean();
                    if(i%2 == 0)
                    {
                        float N = (pos.col(i)-avg).norm();
                        float x = N-0.1;
                        if(x<0.1) acc.col(i) += ok*pow(x,-obs_repel_power)*(pos.col(i)-avg).normalized();
                        //acc.col(i) += ok*pow(0.1,-obs_repel_power)*(pos.col(i)-avg)/N;
                        float drag = gpk*(avg-pos.col(i)).norm();
                        acc.col(i) += 0.5*drag*(avg-pos.col(i)).normalized();
                        acc.col(i) += gdk*(-vel.col(i));
                    }
                }
            }
        }
        return acc;

    }
// updateBehavior: choose update rule by updateMode
    void updateBehavior(MethodTypes type)
    {
        if(!update)  return; // if !update == True, simulation do not update i.e. pause
        if(type == CA_BEHAVE)
        {
            cnt++;
            if(cnt % breed_gap ==0)
            {
                breed(A_pos,A_vel);
                breed(B_pos,B_vel);
                std::cout<<cnt<<'\n';
            }
            attack(A_pos,B_pos,A_vel,B_vel);
            A_pos = Xupdate(A_pos,A_vel);
            A_vel = Vupdate(A_vel,CA_acc(A_pos,A_vel,true));
            B_pos = Xupdate(B_pos,B_vel);
            B_vel = Vupdate(B_vel,CA_acc(B_pos,B_vel));
            std::cout<<"Boids A vs Boid Bs"<<get_A_pos().cols()<<":"<<get_B_pos().cols()<<'\n';
        }
        else
        {
            TVStack acc = TVStack::Zero(dim,n); // init acc
            if(updateMode == 0) // Ex1: Basic Time Integration 25%
            {
                TVStack old_pos = TVStack::Zero(dim,n);
                TVStack old_vel = TVStack::Zero(dim,n);
                old_pos = positions;
                old_vel = velocities;
                acc = getAcc(type,old_pos);
                positions = Xupdate(old_pos,old_vel);
                velocities = Vupdate(old_vel,acc);
            }
            else if(updateMode == 1) // EX2: Advanced Time Integration 25%, symplectic Euler
            {
                positions = Xupdate(positions,velocities);
                acc = getAcc(type, positions); // <--- Updated positions here!
                velocities = Vupdate(velocities,acc);
            }
            else // EX2: Advanced Time Integration 25%, explicit midpoint
            {
                acc = getAcc(type, positions);
                TVStack mid_pos = TVStack::Zero(dim,n);
                TVStack mid_vel = TVStack::Zero(dim,n);
                mid_pos = Xupdate(positions,velocities,true);
                mid_vel = Vupdate(velocities,acc,true);
                acc = getAcc(type,mid_pos);
                positions = Xupdate(positions,mid_vel);
                velocities = Vupdate(velocities,acc);
            }
        }
    }
    void pause()
    {
        update = !update;
    }
    TVStack getPositions()
    {
        return positions;
    }
    TVStack getVelocities()
    {
        return velocities;
    }
    float get_obs_radius()
    {
        return obs_radius;
    }
    TV get_obs_pos()
    {
        return obs_pos;
    }
    TV get_goal_pos()
    {
        return fixed_goal_pos;
    }
    void getMousePos(TV msPos)
    {
        mouse_pos = msPos;
    }
    TVStack get_A_pos()
    {
        return A_pos;
    }
    TVStack get_B_pos()
    {
        return B_pos;
    }

};
#endif
