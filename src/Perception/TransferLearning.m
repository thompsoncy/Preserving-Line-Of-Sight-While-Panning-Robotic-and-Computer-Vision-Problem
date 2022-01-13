
function TransferLearning()
%does the transfer learning 

    digitData = imageDatastore("MyImages", 'IncludeSubfolders',true,'LabelSource','foldernames');
    net = alexnet;
    layersTransfer = net.Layers(1:end-3);
    inputSize = net.Layers(1).InputSize;
    
imageAugmenter = imageDataAugmenter();
augimdsTrain = augmentedImageDatastore(inputSize(1:3),digitData, ...
    'DataAugmentation',imageAugmenter);
disp(augimdsTrain)
    layers = [
    layersTransfer
    fullyConnectedLayer(2,'WeightLearnRateFactor',20,'BiasLearnRateFactor',20)
    softmaxLayer
    classificationLayer];

options = trainingOptions('sgdm', ...
    'MiniBatchSize',10, ...
    'MaxEpochs',6, ...
    'InitialLearnRate',1e-4, ...
    'ValidationFrequency',3, ...
    'ValidationPatience',Inf, ...
    'Verbose',false, ...
    'Plots','training-progress');

    TargetFinderNet = trainNetwork(augimdsTrain,layers,options);

    save TargetFinderNet
end
