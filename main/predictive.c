#include "predictive.h"
#include <math.h>
#include <string.h>

static float bounded(float v, float lo, float hi)
{
    return fminf(hi, fmaxf(lo, v));
}

static float normal_dot(const predictive_solver_t *s, int row, const float *v)
{
    if (row<MPC_HORIZON) return -v[row];
    if (row<2*MPC_HORIZON) return v[row-MPC_HORIZON];
    float sum=0;
    for (int j=0;j<MPC_HORIZON;j++) sum+=s->response[row-2*MPC_HORIZON][j]*v[j];
    return sum;
}

bool predictive_valid(const predictive_model_t *m)
{
    if (!isfinite(m->a1) || !isfinite(m->a2) || !isfinite(m->b1) || !isfinite(m->b2) ||
        !isfinite(m->c) || !isfinite(m->sample_s) || !isfinite(m->move_weight)) return false;
    // Strict Schur stability of z^2-a1*z-a2 (Jury criterion).
    float denominator = 1-m->a1-m->a2;
    if (denominator < .0001f || 1+m->a1-m->a2 < .0001f ||
        1+m->a2 < .0001f || fabsf(m->a2) >= .999f) return false;
    float gain = (m->b1+m->b2)/denominator;
    return m->b1 > 0 && m->b1 <= 20 && fabsf(m->b2) <= 20 &&
           fabsf(m->a1) <= 1.999f && fabsf(m->c) <= 100 &&
           gain >= .1f && gain <= 20 && m->sample_s >= .04f && m->sample_s <= .08f &&
           m->move_weight >= .05f && m->move_weight <= 100;
}

bool predictive_init(predictive_solver_t *s, const predictive_model_t *m)
{
    if (!predictive_valid(m)) return false;
    memset(s, 0, sizeof(*s));
    s->model = *m;
    // Build the input-to-output map once, outside the timed control loop.
    for (int j=0; j<MPC_HORIZON; j++) {
        float y=0, old_y=0, old_u=0;
        for (int k=0; k<MPC_HORIZON; k++) {
            float u=k==j ? 1 : 0;
            float next=m->a1*y+m->a2*old_y+m->b1*u+m->b2*old_u;
            s->response[k][j]=next;
            old_y=y; y=next; old_u=u;
        }
    }
    // Cost: sum (predicted RPM-target)^2 + move_weight*sum (u[k]-u[k-1])^2.
    for (int i=0; i<MPC_HORIZON; i++) {
        for (int j=0; j<MPC_HORIZON; j++) {
            float value=0;
            for (int k=0; k<MPC_HORIZON; k++) value+=s->response[k][i]*s->response[k][j];
            if (i==j) value+=m->move_weight*(i==MPC_HORIZON-1 ? 1 : 2);
            if (i==j+1 || j==i+1) value-=m->move_weight;
            s->hessian[i][j]=value;
        }
    }
    // Invert the positive definite cost Hessian once using a Cholesky factor.
    float lower[MPC_HORIZON][MPC_HORIZON]={0};
    for (int i=0;i<MPC_HORIZON;i++) {
        for (int j=0;j<=i;j++) {
            float value=s->hessian[i][j];
            for (int k=0;k<j;k++) value-=lower[i][k]*lower[j][k];
            if (i==j) {
                if (!isfinite(value) || value<=1e-7f) return false;
                lower[i][j]=sqrtf(value);
            } else lower[i][j]=value/lower[j][j];
        }
    }
    for (int column=0;column<MPC_HORIZON;column++) {
        float temp[MPC_HORIZON]={0};
        for (int i=0;i<MPC_HORIZON;i++) {
            float value=i==column ? 1 : 0;
            for (int j=0;j<i;j++) value-=lower[i][j]*temp[j];
            temp[i]=value/lower[i][i];
        }
        for (int i=MPC_HORIZON-1;i>=0;i--) {
            float value=temp[i];
            for (int j=i+1;j<MPC_HORIZON;j++) value-=lower[j][i]*s->inverse[j][column];
            s->inverse[i][column]=value/lower[i][i];
        }
    }
    s->positive_response=true;
    for (int row=0;row<MPC_CONSTRAINTS;row++) {
        for (int i=0;i<MPC_HORIZON;i++) {
            if (row<MPC_HORIZON) s->inverse_normal[row][i]=-s->inverse[i][row];
            else if (row<2*MPC_HORIZON) s->inverse_normal[row][i]=s->inverse[i][row-MPC_HORIZON];
            else {
                float value=0;
                for (int j=0;j<MPC_HORIZON;j++) {
                    float response=s->response[row-2*MPC_HORIZON][j];
                    if (response<0) s->positive_response=false;
                    value+=s->inverse[i][j]*response;
                }
                s->inverse_normal[row][i]=value;
            }
        }
    }
    s->predicted_peak=s->prediction_limit=s->constraint_violation=-1;
    return true;
}

bool predictive_step(predictive_solver_t *s, float y, float old_y, float old_u,
                     float disturbance, float target, float maximum, float *pwm)
{
    s->sweeps=0; s->residual=INFINITY;
    s->prediction_limit=s->constraint_violation=s->predicted_peak=-1;
    if (!isfinite(y) || !isfinite(old_y) || !isfinite(old_u) || !isfinite(disturbance) ||
        !isfinite(target) || !isfinite(maximum) || maximum<=0 || maximum>80) return false;
    float previous_control=old_u;
    float free_output[MPC_HORIZON], linear[MPC_HORIZON];
    for (int k=0; k<MPC_HORIZON; k++) {
        float next=s->model.a1*y+s->model.a2*old_y+s->model.b2*old_u+s->model.c+disturbance;
        if (!isfinite(next)) return false;
        free_output[k]=next-target;
        old_y=y; y=next; old_u=0; // Future controls are represented by response[].
    }
    for (int j=0; j<MPC_HORIZON; j++) {
        linear[j]=0;
        for (int k=0; k<MPC_HORIZON; k++) linear[j]+=s->response[k][j]*free_output[k];
    }
    // The first move is relative to the PWM that was actually held last interval.
    linear[0]-=s->model.move_weight*previous_control;
    for (int j=0; j<MPC_HORIZON-1; j++) s->plan[j]=bounded(s->plan[j+1],0,maximum);
    s->plan[MPC_HORIZON-1]=bounded(s->plan[MPC_HORIZON-1],0,maximum);
    for (int sweep=1; sweep<=MPC_MAX_SWEEPS; sweep++) {
        for (int j=0; j<MPC_HORIZON; j++) {
            float gradient=linear[j];
            for (int k=0; k<MPC_HORIZON; k++) gradient+=s->hessian[j][k]*s->plan[k];
            if (!isfinite(gradient) || !isfinite(s->hessian[j][j]) || s->hessian[j][j]<=0) return false;
            s->plan[j]=bounded(s->plan[j]-gradient/s->hessian[j][j],0,maximum);
        }
        float residual=0;
        for (int j=0; j<MPC_HORIZON; j++) {
            float gradient=linear[j];
            for (int k=0; k<MPC_HORIZON; k++) gradient+=s->hessian[j][k]*s->plan[k];
            if (!isfinite(gradient)) return false;
            float correction=bounded(s->plan[j]-gradient/s->hessian[j][j],0,maximum)-s->plan[j];
            residual=fmaxf(residual,fabsf(correction));
        }
        s->sweeps=sweep; s->residual=residual;
        if (!isfinite(residual)) return false;
        if (residual <= .03f) {
            s->predicted_peak=-INFINITY;
            for (int k=0;k<MPC_HORIZON;k++) {
                float speed=free_output[k]+target;
                for (int j=0;j<MPC_HORIZON;j++) speed+=s->response[k][j]*s->plan[j];
                s->predicted_peak=fmaxf(s->predicted_peak,speed);
            }
            *pwm=s->plan[0];
            return isfinite(*pwm);
        }
    }
    return false; // No silent fallback to a different controller.
}

// Solve the active-constraint Gram system, with partial pivoting. All arrays are
// fixed-size; this scratch space plus the caller's vectors stays below 1KB.
static bool active_multipliers(const predictive_solver_t *s, const int *working, int count,
                               const float *free_step, float *multipliers)
{
    float matrix[MPC_HORIZON][MPC_HORIZON+1];
    for (int i=0;i<count;i++) {
        for (int j=0;j<count;j++)
            matrix[i][j]=normal_dot(s,working[i],s->inverse_normal[working[j]]);
        matrix[i][count]=normal_dot(s,working[i],free_step);
    }
    for (int k=0;k<count;k++) {
        int pivot=k;
        for (int i=k+1;i<count;i++) if (fabsf(matrix[i][k])>fabsf(matrix[pivot][k])) pivot=i;
        if (!isfinite(matrix[pivot][k]) || fabsf(matrix[pivot][k])<1e-8f) return false;
        for (int j=k;j<=count;j++) {
            float temp=matrix[k][j];matrix[k][j]=matrix[pivot][j];matrix[pivot][j]=temp;
        }
        for (int i=k+1;i<count;i++) {
            float scale=matrix[i][k]/matrix[k][k];
            for (int j=k;j<=count;j++) matrix[i][j]-=scale*matrix[k][j];
        }
    }
    for (int i=count-1;i>=0;i--) {
        float value=matrix[i][count];
        for (int j=i+1;j<count;j++) value-=matrix[i][j]*multipliers[j];
        multipliers[i]=value/matrix[i][i];
        if (!isfinite(multipliers[i])) return false;
    }
    return true;
}

bool predictive_step_capped(predictive_solver_t *s, float y, float old_y, float old_u,
                           float disturbance, float target, float maximum,
                           float margin, float *pwm)
{
    s->sweeps=0; s->residual=INFINITY; s->result=PREDICTIVE_INVALID;
    s->predicted_peak=s->constraint_violation=-1; s->prediction_limit=target-margin;
    if (!isfinite(y) || !isfinite(old_y) || !isfinite(old_u) || !isfinite(disturbance) ||
        !isfinite(target) || !isfinite(maximum) || !isfinite(margin) ||
        maximum<=0 || maximum>80 || margin<0 || margin>5 || target-margin<=0) return false;
    if (!s->positive_response) {s->result=PREDICTIVE_UNSUPPORTED;return false;}
    float previous_control=old_u;
    float free_output[MPC_HORIZON],linear[MPC_HORIZON],bounds[MPC_CONSTRAINTS];
    // A 0.005 RPM numerical reserve keeps acceptance tolerance inside the requested cap.
    float tightened_limit=s->prediction_limit-.005f;
    for (int k=0;k<MPC_HORIZON;k++) {
        float next=s->model.a1*y+s->model.a2*old_y+s->model.b2*old_u+s->model.c+disturbance;
        if (!isfinite(next)) {s->result=PREDICTIVE_NUMERICAL;return false;}
        free_output[k]=next;old_y=y;y=next;old_u=0;
        bounds[k]=0; bounds[MPC_HORIZON+k]=maximum;
        bounds[2*MPC_HORIZON+k]=tightened_limit-next;
        // Since all response coefficients are nonnegative, zero future PWM
        // minimizes every predicted speed. A violating zero plan proves infeasibility.
        if (bounds[2*MPC_HORIZON+k]<0) {s->result=PREDICTIVE_INFEASIBLE;return false;}
    }
    for (int j=0;j<MPC_HORIZON;j++) {
        linear[j]=0;
        for (int k=0;k<MPC_HORIZON;k++) linear[j]+=s->response[k][j]*(free_output[k]-target);
        s->plan[j]=0; // A feasible seed, including after a target change.
    }
    linear[0]-=s->model.move_weight*previous_control;
    int working[MPC_HORIZON],count=0;
    bool active[MPC_CONSTRAINTS]={false};
    for (int iteration=1;iteration<=MPC_ACTIVE_ITERATIONS;iteration++) {
        float gradient[MPC_HORIZON],step[MPC_HORIZON],free_step[MPC_HORIZON],lambda[MPC_HORIZON]={0};
        for (int i=0;i<MPC_HORIZON;i++) {
            gradient[i]=linear[i];
            for (int j=0;j<MPC_HORIZON;j++) gradient[i]+=s->hessian[i][j]*s->plan[j];
            if (!isfinite(gradient[i])) {s->result=PREDICTIVE_NUMERICAL;return false;}
        }
        for (int i=0;i<MPC_HORIZON;i++) {
            free_step[i]=0;
            for (int j=0;j<MPC_HORIZON;j++) free_step[i]-=s->inverse[i][j]*gradient[j];
            step[i]=free_step[i];
        }
        if (!active_multipliers(s,working,count,free_step,lambda)) {
            s->result=PREDICTIVE_NUMERICAL;return false;
        }
        float norm=0;
        for (int i=0;i<MPC_HORIZON;i++) {
            for (int j=0;j<count;j++) step[i]-=s->inverse_normal[working[j]][i]*lambda[j];
            if (!isfinite(step[i])) {s->result=PREDICTIVE_NUMERICAL;return false;}
            norm=fmaxf(norm,fabsf(step[i]));
        }
        s->sweeps=iteration; s->residual=norm;
        if (norm<=.001f) {
            int remove=-1;float smallest=-.0001f;
            for (int j=0;j<count;j++) if (lambda[j]<smallest) {smallest=lambda[j];remove=j;}
            if (remove>=0) {
                active[working[remove]]=false;
                for (int j=remove;j<count-1;j++) working[j]=working[j+1];
                --count;continue;
            }
            for (int i=0;i<MPC_HORIZON;i++) s->plan[i]=bounded(s->plan[i],0,maximum);
            // Check every original inequality again, independently of the working set.
            for (int row=0;row<MPC_CONSTRAINTS;row++) {
                if (normal_dot(s,row,s->plan)-bounds[row]>.002f) {
                    s->result=PREDICTIVE_NUMERICAL;return false;
                }
            }
            s->predicted_peak=-INFINITY;
            for (int k=0;k<MPC_HORIZON;k++) {
                float predicted=free_output[k]+normal_dot(s,2*MPC_HORIZON+k,s->plan);
                s->predicted_peak=fmaxf(s->predicted_peak,predicted);
            }
            s->constraint_violation=fmaxf(0,s->predicted_peak-s->prediction_limit);
            if (!isfinite(s->predicted_peak) || s->constraint_violation>0) {
                s->result=PREDICTIVE_NUMERICAL;return false;
            }
            // Roundoff on box boundaries is harmless only toward lower PWM.
            *pwm=bounded(s->plan[0],0,maximum);
            s->result=PREDICTIVE_OK;return isfinite(*pwm);
        }
        float alpha=1;int blocker=-1;
        for (int row=0;row<MPC_CONSTRAINTS;row++) {
            if (active[row]) continue;
            float direction=normal_dot(s,row,step);
            if (direction>1e-6f) {
                float slack=bounds[row]-normal_dot(s,row,s->plan);
                if (slack<-.002f) {s->result=PREDICTIVE_NUMERICAL;return false;}
                float candidate=fmaxf(0,slack)/direction;
                if (candidate<alpha) {alpha=candidate;blocker=row;}
            }
        }
        for (int i=0;i<MPC_HORIZON;i++) s->plan[i]+=alpha*step[i];
        if (blocker>=0) {
            if (count==MPC_HORIZON) {s->result=PREDICTIVE_NUMERICAL;return false;}
            working[count++]=blocker;active[blocker]=true;
        }
    }
    s->result=PREDICTIVE_ITERATION_LIMIT;return false;
}
