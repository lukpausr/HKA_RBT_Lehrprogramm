% Test script for ERP

function R = calculate_rotation_matrix_erp(q)
    R = [ 2*q(1)^2 - 1 + 2*q(2)^2,     2*q(2)*q(3) - 2*q(1)*q(4),     2*q(2)*q(4) + 2*q(1)*q(3)  ;
          2*q(2)*q(3) + 2*q(1)*q(4),   2*q(1)^2 - 1 + 2*q(3)^2,       2*q(3)*q(4) - 2*q(1)*q(2)  ;
          2*q(2)*q(4) - 2*q(1)*q(3),   2*q(3)*q(4) + 2*q(1)*q(2),     2*q(1)^2 - 1 + 2*q(4)^2   ];
end

function q = calculate_q_from_erp_rotation_matrix(R)
    q0 = sqrt( ( R(1,1) + R(2,2) + R(3,3) + 1 ) / 4 );
    q1 = ( R(3,2) - R(2,3) ) / (4 * q0);
    q2 = ( R(1,3) - R(3,1) ) / (4 * q0);
    q3 = ( R(2,1) - R(1,2) ) / (4 * q0);
    q = [ q0 ; q1 ; q2 ; q3 ];
end







vector = [ 1 ; 0.131 ; 0];
tranimate(eye(3));
hold on;

% Define Angle
angle = 60;
amount_of_full_rotations_until_plot = 100;
total_amount_of_rotations = 10000;

rotations_per_360_deg = 360 / angle;
angle_rad = deg2rad(angle);

R_xyzuvw = rotz(angle_rad);
q = calculate_q_from_erp_rotation_matrix(R_xyzuvw);
R_erp = calculate_rotation_matrix_erp(q);
R_erp = round(R_erp,6);

% Compare Robotic Toolbox vs ERP

% Robotic Toolbox
for z=1:total_amount_of_rotations 

    for u=1:rotations_per_360_deg
        if(mod(z,amount_of_full_rotations_until_plot)==0)
            plot_arrow([0;0;0],vector,'r')
            disp("printing round" + z)
        end
        vector = R_erp * vector;
    end

end


% Idee Speicherplatz

return;


% Idee Rechenzeit
% Als externe Funktion auslagern damit die Variablen nicht dauerhaft
% gespeichert werden - wer weiß was Matlab damit macht?
rotationen_xyz = [45 ; 30 ; 60];

R_x = rotx(rotationen_xyz(1));
R_y = roty(rotationen_xyz(2));
R_z = rotz(rotationen_xyz(3));
R_ges = R_z * (R_y * R_x)
% R_ges_2 = R_z * (R_x * R_y)



% Wie wird bei ERP vorgegangen?
% q0-3 sind gegeben, daraus erechnet sich die Rotationsmatrix und wiederum
% daraus ein verdrehter Vektor

q = calculate_q_from_erp_rotation_matrix(R_ges);
R = calculate_rotation_matrix_erp(q)

vector = [1; 1; 1];
r1 = R_ges * vector;
r2 = R * vector;

tranimate(eye(3));
hold on;
plot_arrow([0;0;0],r1,'r');
plot_arrow([0;0;0],r2,'b');




% Start Erfassung Rechenzeit "Robotic Toolbox"
tic;

vector_rechenzeit = [1.5 ; 2.3 ; 4.1];
% Start Rechnung
for iteration=1:10000
    R_x = rotx(rotationen_xyz(1));
    vector_rechenzeit = R_x * vector_rechenzeit;
    R_y = roty(rotationen_xyz(2));
    vector_rechenzeit = R_y * vector_rechenzeit;
    R_z = rotz(rotationen_xyz(3));
    vector_rechenzeit = R_z * vector_rechenzeit;
end

% Ende Erfassung Rechenzeit
elapsed_time_rtb = toc

% Start Erfassung Rechenzeit "ERP"
tic;

vector_rechenzeit = [1.5 ; 2.3 ; 4.1];
% Start Rechnung
for iteration=1:10000
    R = calculate_rotation_matrix_erp(q);
    vector_rechenzeit = R * vector_rechenzeit;
end

% Ende Erfassung Rechenzeit
elapsed_time_erp = toc