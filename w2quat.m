function quat = w2quat(w, t)

persistent last_q
persistent last_t

if isempty(last_q)
    last_q = [1;
              0;
              0;
              0];  % initial rotation alligned with the orbit frame
end
if isempty(last_t)
    last_t = 0;
end

delta_t = t - last_t;    % hard set in fixed step size
n = last_q(1,1);
e = [   last_q(2,1);
        last_q(3,1);
        last_q(4,1)];
e_skew = [   0      -e(3)   e(2);
             e(3)   0       e(1);
            -e(2)   e(1)    0];

T_q = [-transpose(e);
        n*eye(3) + e_skew];

q_dot = 1/2*T_q*w;
quat = last_q + q_dot * delta_t;

last_q = quat;
last_t = t;