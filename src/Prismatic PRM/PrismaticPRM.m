function PrismaticPRM()
    close all;
    
	L(1) = Link([0 0 0 1.571]);
    L(2) = Link([0 0 0 -1.571]);
    L(3) = Link([0 0.4318 0 -1.571]);
    L(4) = Link([0,0,0,0,1],'standard');
    rob = SerialLink(L, 'name', 'robot');
    s = [2.8600   -2.2400   -3.1400   -3.1400];
    g = [0 -2 0 0];
    goallocation = rob.fkine(g);
    goallocation = [goallocation(1,4) goallocation(2,4) goallocation(3,4)];
    mi = [-pi,-pi,-pi,-pi];
    ma = [pi/2,0,0,0];
    sphereCenter = [0;0.5;0];
    sphereCenter2 = [0.7;0.7;0];
    sphereRadius = 0.2;
        
    % Plot robot and sphere
    rob.plot(s,'workspace',[-1,1,-1,1,-1,1]);
    hold on;	
    drawSphere(sphereCenter,sphereRadius);
    
    drawSphere(sphereCenter2,sphereRadius);

    % Warning, this code can be very slow to run
    
    prmNumSamples = 100;
    d = size(mi, 2); % Number of dimensions
    samples = zeros(prmNumSamples, d);
    
    samples(1, 1:d) = s; % Keep start and goal as first and second entries
    samples(2, 1:d) = g;
    
    nextRow = 3;
    while nextRow < prmNumSamples
        sample = (ma - mi).*rand(1,d) + mi;
        forwardkinmatics = rob.fkine(sample);
        
        fk = [forwardkinmatics(1,4), forwardkinmatics(2,4), forwardkinmatics(3,4)];
        if norm(fk - goallocation) > .1
            continue
        end
        if robotCollision(rob, sample, sphereCenter, sphereRadius)
            continue
        end
        if robotCollision(rob, sample, sphereCenter2, sphereRadius)
            continue
        end
        samples(nextRow, 1:d) = sample;
        nextRow = nextRow + 1;
    end
    
    adjacencyMat = zeros(prmNumSamples, prmNumSamples);

    for i = 1:prmNumSamples % Create adjacency matrix
        sample1 = samples(i, 1:d);
        for j = 1:prmNumSamples
            sample2 = samples(j, 1:d);
            if i > j
                adjacencyMat(i, j) = adjacencyMat(j, i);
            elseif not(checkEdge(rob, sample1, sample2, sphereCenter, sphereRadius)) && ...
                not(checkEdge(rob, sample1, sample2, sphereCenter2, sphereRadius))
                adjacencyMat(i, j) = norm(sample2 - sample1);
            end
        end
    end
    
    G = graph(adjacencyMat);
    path = shortestpath(G,1,2);
    
    qMilestones = zeros(size(path, 2), d);
    for v = 1:size(path, 2)
        sample = samples(path(1, v), 1:d);
        qMilestones(v, 1:d) = sample;
    end
    
    % Plot robot following path
    d = 0.05;
    traj = [];
    for i=2:size(qMilestones,1)
        delta = qMilestones(i,:) - qMilestones(i-1,:);
        m = max(floor(norm(delta) / d),1);
        vec = linspace(0,1,m);
        leg = repmat(delta',1,m) .* repmat(vec,size(delta,2),1) + repmat(qMilestones(i-1,:)',1,m);
        traj = [traj;leg'];
    end
    rob.plot(traj);
end

function drawSphere(position,diameter)

%     diameter = 0.1;
    [X,Y,Z] = sphere;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    surf(X,Y,Z);
    %~ shading flat

end