clear;
close all;

image = im2double(imread('uno-test-28.jpg'));
skin  = im2double(imread('sfondo_20.jpg'));

[r,c,ch] = size(skin);

pixs = reshape(skin,r*c,ch);
m = mean(pixs);
v = std(pixs);
k = 2.3;

%range che se veri => pixel di background
mr = (image(:,:,1) <= m(1)+k*v(1)) & (image(:,:,1) >= m(1)-k*v(1));
mg = (image(:,:,2) <= m(2)+k*v(2)) & (image(:,:,2) >= m(2)-k*v(2));
mb = (image(:,:,3) <= m(3)+k*v(3)) & (image(:,:,3) >= m(3)-k*v(3));

%faccio l'and tra i valori dei range => tutti e 3 veri => pixel background
predicted = ~ (mr & mg & mb);

%vengono presi solo i pixel > 0 => essendo un immagine binaria prende solo
%i pixel bianchi che corrispondono a quelli di pelle
%gt = imread('test1-gt.png')>0;

%funzione che ritorna l'accuratezza dei pixel di pelle trovati su quelli
%presenti => MATRICE DI CONFUSIONE (va guardate la diagonale per avere
%un'idea precisa dell'accuratezza
%cm = confmat(gt,predicted);

show_result(image,predicted);