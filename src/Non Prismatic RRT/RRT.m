% simple RRT with extra checks when sampling to keep line of sight
function RRT()
    

	L(1) = Link([0 0 0 1.571]);
    
    
    L(2) = Link([0 0 0 -1.571]);
    
    
    L(3) = Link([0 0.4318 0 -1.571]);
    
    
    L(4) = Link([0 0.4318 0 -1.571]);
    
    close all;
    rob = SerialLink(L, 'name', 'robot');
    
    start = [2.8600   -2.2400   -3.1400   -3.1400];
    
    goal = [0 -1 0 0];
   
    
    mi = [-pi/2,-pi,0,0];
    ma = [pi/2,0,0,0];
    sphereCenter = [0;.4;0];
    sphereCenter2 = [0;-.4;0];
    sphereCenter3 = [.7;0;.5];
    sphereRadius = 0.2;
    
    % Plot robot and sphere
    rob.plot(start,'workspace',[-1,1,-1,1,-1,1]);
    hold on;	
    drawSphere(sphereCenter,sphereRadius);
    
    drawSphere(sphereCenter2,sphereRadius);
    
    drawSphere(sphereCenter3,sphereRadius);

    tree = start; % the rrt tree
    treesources = [-1]; % the list where each tree node came from
    qMilestones = -1;
    while qMilestones == -1 % go until a path to goal is found
        n = 100;
        m = size(mi, 2); %dimensions
        samplenodes = zeros(n, m);
        for r = 1:m
            samplenodes(:,r) = (ma(r)-mi(r)).*rand(n,1) + mi(r);
        end % genetated random samples withing the min and max
        toberemoved = [];
        for c = 1:n 
            if robotCollision(rob, samplenodes(n,:), sphereCenter, sphereRadius)
                toberemoved = [c , toberemoved];
            end
            if robotCollision(rob, samplenodes(n,:), sphereCenter2, sphereRadius)
                toberemoved = [c , toberemoved];
            end
           if ~checkEdge(rob,  samplenodes(n,:), goal, sphereCenter, sphereRadius) && ~checkEdge(rob,samplenodes(n,:), goal, sphereCenter2, sphereRadius)
                toberemoved = [c , toberemoved];
           end
        end % makes sure samples are valid
        samplenodes(toberemoved,:) = [];
        samplenodes = [samplenodes ; goal]; 
        for snode = 1:size(samplenodes,1) % run through all samples end on the goal
            closestdist = vecnorm(tree(1,:) - samplenodes(snode,:));
            closestnode = 1;
            for treenum = 2:size(tree,1) % find node in tree that is closest to sample
                dist = vecnorm(tree(treenum,:) - samplenodes(snode,:));
                if dist < closestdist
                    closestdist = dist;
                    closestnode = treenum;
                end
            end
            if  ~checkEdge(rob, tree(closestnode,:), samplenodes(snode,:), sphereCenter, sphereRadius) && ~checkEdge(rob, tree(closestnode,:), samplenodes(snode,:), sphereCenter2, sphereRadius) && ~checkEdge(rob, tree(closestnode,:), samplenodes(snode,:), sphereCenter3, sphereRadius) 
                if snode == size(samplenodes,1)
                   qMilestones = goal; % path has been found
                end
                tree = [tree ; samplenodes(snode,:)]; % add connected node to tree
                treesources = [treesources, closestnode];
            end  
        end
    end
    lastnode = size(tree,1);
    while lastnode ~=1 % get the path
        qMilestones = [tree(treesources(lastnode),:);qMilestones];
        lastnode = treesources(lastnode);
    end
        
    % Plot robot following path
    d = 0.05;;
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
