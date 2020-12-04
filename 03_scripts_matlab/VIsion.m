%% codigo vision de maquina
% configuracion previa
clc;
clear;

%% Configuracion previa 
% Se crea un objeto cam

cam = webcam('USB2.0 Camera: USB2.0 Camera');

%% Adquision imagen
%  Se llama la funcion que toma una foto de la fresa y la guarda en la
%  variable img
img = snapshot(cam);

%% Verificacion Foto
% Se muestra la imagen mostrada
%figure(1)
%imshow(img)

%% Preproceso
% se pasa la transforma la escala de grises

hsvImg = rgb2gray(img);

I=rgb2gray(img);

% Se aplica filtro Gauseano con un nivel 
B4 = imgaussfilt(I,4); 

% Se aplica a cada una un tresholding con limite en 0.8
TH= im2bw(B4,0.165);
% se aplica un filtro morfologico 
se = strel('arbitrary',15);
closeBW = imclose(TH,se);
closeBW  = imfill(closeBW ,'holes');

se = strel('arbitrary',5);
afterOpening = imopen(closeBW,se);
%% sacar el mas alto 
BW2 = bwareafilt(closeBW,1);
BW2 = imfill(BW2,'holes');
border = edge(BW2);
% blob analysis
blobAnalyzer = vision.BlobAnalysis('MajorAxisLengthOutputPort', true,'MinorAxisLengthOutputPort', true, 'OrientationOutputPort', true);
[area, centroid, bbox, major, minor, orientation] = step(blobAnalyzer, BW2);
final = BW2 ;
figure(5)
subplot(2,3,1); imshow(img)
subplot(2,3,2); imshow(B4)
subplot(2,3,3); imshow(TH)
subplot(2,3,4); imshow(BW2)
subplot(2,3,5); imshow(border)
subplot(2,3,6); imshow(img)
% line([x1,x2],[y1,y2]), not line([x1,y1],[x2,y2]);
line([0 700], [(major-minor) (major-minor)]);