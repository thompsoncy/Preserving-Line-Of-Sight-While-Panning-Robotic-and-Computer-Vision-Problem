%
% Compute a cell array of adjacency matrices.
% input: distanceMat -> nxn pairwise distance matrix found in Q1.
%        numSamplesMat -> kxl matrix of numSamples
%        radiusMatr -> kxl matrix of radiuses
% output: adjacencyCellArray -> kxl cell array of distance matrices where
%                               each element contains the adjacency matrix
%                               for the corresponding numSamples and
%                               radius.
% 
function adjacencyCellArray = getAdjacency(distanceMat, numSamplesMat, radiusMat)

    if size(distanceMat,1) < max(max(numSamplesMat))
        error('Size of distanceMat must be at least as large as max num of samples in numSamplesMat')
    end
    
    adjacencyCellArray = cell(size(numSamplesMat));    
    for i = 1:size(numSamplesMat,1)
        for j = 1:size(numSamplesMat,2)
            radius = radiusMat(i, j);
            numSamples = numSamplesMat(i, j);
            adjacencyCellArray{i,j} = Q2(distanceMat, numSamples, radius);
        end
    end

end
