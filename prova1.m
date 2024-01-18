clear;
close all;

immagine_originale = im2double(imread('uno-test-19.jpg'));
% Leggi un'immagine in scala di grigi
image = rgb2gray(im2double(immagine_originale));

% Converte l'immagine in binaria
soglia_nick = adaptthresh(image, 0.55, 'NeighborhoodSize', 15);
binaryImage = imbinarize(image, soglia_nick);

% Etichetta le regioni connesse nell'immagine binaria
labels = bwlabel(binaryImage); 

% Calcolo delle proprietà delle nuove regioni separate
stats = regionprops(labels, 'BoundingBox','Area','MajorAxisLength','ConvexHull');
largeRegionsIndices = find([stats.Area] > 4000 & [stats.MajorAxisLength] < 1000);

%applico morfologia matematica se ci sono carte sovrapposte
for i = 1:numel(stats)
    if i <= length(stats)
    if stats(i).Area > 20000
        % Applicazione di apertura morfologica per separare le regioni sovrapposte
        separationImage = imopen(binaryImage, strel('square', 3));
        % Etichettatura delle regioni connesse nella nuova immagine dopo erosione
        separationLabels = bwlabel(separationImage);

        % Calcolo delle proprietà delle nuove regioni separate
        stats = regionprops(separationLabels, 'BoundingBox','Area','MajorAxisLength','ConvexHull');
        % Loop attraverso le nuove regioni separate dopo erosione
        for j = 1:numel(stats)
            if(stats(j).Area>4000)
            % BoundingBox della regione separata
            rectangle('Position', stats(j).BoundingBox, 'EdgeColor', 'r', 'LineWidth', 3);
            end
        end
    end
    end
end

% Visualizzazione dell'immagine binaria originale
    figure;
    imshow(binaryImage);
    title('Immagine Binaria Originale');



%conta quante carte ci sono (label con area < 3000 e lunghezza max 1000)
largeRegionsIndices = find([stats.Area] > 4000 & [stats.MajorAxisLength] < 1000);
largeCards = numel(largeRegionsIndices);

% Visualizza le bounding boxes delle regioni su immagine a colori
%figure;
im = immagine_originale;
%imshow(im);
%hold on;

for i = 1:numel(stats)
    if ismember(i, largeRegionsIndices)
         %BoundingBox
         rectangle('Position', stats(i).BoundingBox, 'EdgeColor', 'r', 'LineWidth', 3); % Aumenta lo spessore a 3 (puoi regolare questo valore)
         
         % Estrai la regione di interesse usando la bounding box
         regionOfInterest = imcrop(image, stats(i).BoundingBox);
         
         % Applica Canny Edge Detection alla regione di interesse
         edgeImage = edge(regionOfInterest, 'Canny', 0.5);
         
         % Dilata gli edge per aumentarne lo spessore
         se = strel('disk', 3); % Puoi regolare la dimensione del disco per ottenere lo spessore desiderato
         edgeImage = imdilate(edgeImage, se);
         
         % Sovrapponi i bordi trovati sull'immagine originale
         [rows, cols] = size(edgeImage);
         
         % Calcola gli indici corretti per sovrapporre edgeImage su im
         rowsIdx = round(stats(i).BoundingBox(2)):(round(stats(i).BoundingBox(2)) + rows - 1);
         colsIdx = round(stats(i).BoundingBox(1)):(round(stats(i).BoundingBox(1)) + cols - 1);
         
         % Converti edgeImage in double prima di moltiplicare
         edgeImage = double(edgeImage);
         
         % Sostituisci i pixel corrispondenti nell'immagine originale
         im(rowsIdx, colsIdx) = im(rowsIdx, colsIdx) .* ~edgeImage + 1.0 .* edgeImage; % Use .* for element-wise multiplication
    end
end

% Visualizza l'immagine con gli edge dilatati sulle regioni di interesse
%figure;
%imshow(im);
%title('Edge Dilatati sulle Regioni di Interesse');

% Inizializza una cella per memorizzare le coordinate degli edge dilatati
edgeCoordinates = cell(1, numel(stats));

for i = 1:numel(stats)
    if ismember(i, largeRegionsIndices)
         % Estrai la regione di interesse usando la bounding box
         regionOfInterest = imcrop(image, stats(i).BoundingBox);
         
         % Applica Canny Edge Detection alla regione di interesse
         edgeImage = edge(regionOfInterest, 'Canny', 0.5);
         
         % Dilata gli edge per aumentarne lo spessore
         se = strel('disk', 3);
         edgeImage = imdilate(edgeImage, se);
         
         % Sovrapponi i bordi trovati sull'immagine nera
         edgeImageOnly(round(stats(i).BoundingBox(2)):(round(stats(i).BoundingBox(2)) + size(edgeImage, 1) - 1), ...
             round(stats(i).BoundingBox(1)):(round(stats(i).BoundingBox(1)) + size(edgeImage, 2) - 1)) = edgeImage;
         
         % Trova le coordinate degli edge dilatati
         [rows, cols] = find(edgeImage);
         % Trasla le coordinate nella posizione dell'immagine originale
         rows = rows + round(stats(i).BoundingBox(2)) - 1;
         cols = cols + round(stats(i).BoundingBox(1)) - 1;
         % Memorizza le coordinate nella cella
         edgeCoordinates{i} = [rows, cols];
    end
end

% Inizializza un'immagine nera delle stesse dimensioni dell'immagine originale
resultImage = zeros(size(im));

% Loop attraverso le regioni di interesse
for i = 1:numel(stats)
    if ismember(i, largeRegionsIndices)
        % Ottieni le coordinate degli edge dilatati per questa regione
        coordinates = edgeCoordinates{i};
        
        % Estrai la regione di interesse usando la convex hull
        convexHull = roipoly(immagine_originale, stats(i).ConvexHull(:, 1), stats(i).ConvexHull(:, 2));
        regionOfInterest = immagine_originale.*convexHull;

        % Calcola gli indici corretti per sovrapporre la regione di interesse su resultImage
        rowsIdx = round(stats(i).BoundingBox(2)):(round(stats(i).BoundingBox(2)) + size(regionOfInterest, 1) - 1);
        colsIdx = round(stats(i).BoundingBox(1)):(round(stats(i).BoundingBox(1)) + size(regionOfInterest, 2) - 1);
        
        % Limita gli indici per evitare overflow
        rowsIdx = max(1, rowsIdx);
        colsIdx = max(1, colsIdx);
        rowsIdx = min(size(resultImage, 1), rowsIdx);
        colsIdx = min(size(resultImage, 2), colsIdx);
        
        % Assegna i valori della regione di interesse a resultImage usando la logica
        resultImage(rowsIdx, colsIdx, :) = resultImage(rowsIdx, colsIdx, :) + regionOfInterest(rowsIdx, colsIdx, :);
        singleCard= regionOfInterest(rowsIdx, colsIdx, :);
  
        %figure,
        %imshow(singleCard);
    end
end

% Visualizza l'immagine risultante
figure;
imshow(resultImage);
title('Immagine finale');

save('workspace3.mat','singleCard');
