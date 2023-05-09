
classical = Classical_optimizer();


% vodenje po trajektoriji baze

    qa=[ 0 ...    % roka
        pi/4 ...
        0 ...
        -pi/3 ...
        0 ...
        1.8675 ...
        0];
    qb=[0 1 pi/2]+ [0 -.4 pi/4]*1 ; % lega baza z dodnim pogreskom

    a=1;
    qa(a)=qa(a)-pi/2*0;
 
    q0=[qa,qb];

classical.controlBase(q0);

