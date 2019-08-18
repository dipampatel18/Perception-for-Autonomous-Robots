%%
%% Traffic Sign Classification

% Training of Frames

TrainingFile = fullfile('P4_Submission', 'TSR', 'Input', 'Training');
TrainingSet = imageSet(TrainingFile, 'recursive');

TrainingFeatures = [];
TrainingLabels   = [];

for i = 1:numel(TrainingSet)

        numImages = TrainingSet(i).Count;
        hog = [];

        for j = 1:numImages
            img = read(TrainingSet(i), j);
            i
            j

% Resize Image to 64x64
            img = im2single(imresize(img,[64 64]));

% Get HOG Features

            hog_cl = vl_hog(img, 4);
            [hog_1, hog_2] = size(hog_cl);
            dim = hog_1*hog_2;
            hog_cl_trans = permute(hog_cl, [2 1 3]);
            hog_cl = reshape(hog_cl_trans, [1 dim]);
            hog(j,:) = hog_cl;

        end

   labels = repmat(TrainingSet(i).Description, numImages, 1);

   TrainingFeatures = [TrainingFeatures; hog];
   TrainingLabels = [TrainingLabels; labels];

end

% SVM

classifier = fitcecoc(TrainingFeatures, TrainingLabels);

filename = 'Classifier.mat'
save ('Classifier.mat')
