%% considerations
fs = 1e3;
T = 1/fs;

% robot's properties
L = 0.1;    % robot's radium
r = 0.02;   % wheel's radium
alpha1_deg = 45;    % WÔF
alpha2_deg = 135;
m = 0.8;    % mass
Icom = rand;    % moment of inertia CoM

% motors' properties
K = rand;   % torque constant
Jm = rand;  % inertia
Bm = rand;  % viscous friction coeff
R = 50;

% reduction's properties
N = 10;     % reduction factor
eta = 0.9;  % efficiency

% wheels' properties
Jw = rand;  % inertia
Bw = rand;  % viscous friction coeff

% Jeq and Beq
Jeq = Jm * N^2 * eta + Jw;
Beq = Bm * N^2 * eta + Bw;

%% matrixes
% velocities: wheels 2 robots
M = [-sind( alpha1_deg), cosd( alpha1_deg), L;
     -sind( alpha2_deg), cosd( alpha2_deg), L;
     -sind(-alpha2_deg), cosd(-alpha2_deg), L;
     -sind(-alpha1_deg), cosd(-alpha1_deg), L];
mtx.rMplus = r * pinv(M);

% useful ones
Cv = (Beq + K^2 * N^2 * eta / R) * eye(4);
k = N * eta * K / R;
Id_a = m * r^2 / 4;
Id_b = Icom * r^2 / 16 / L^2;
Id = Id_a + Id_b;
Iadj = Id_b;
Iop = Id_b - Id_a;
Htau_aux = diag(ones(3,1),1) * Iadj + diag(ones(2,1),2) * Iop + diag(Iadj,3);
Htau = Htau_aux' + eye(4) * Id + Htau_aux;
H = Htau + Jeq * eye(4);

% A and B
mtx.A = exp(inv(H) * Cv)




