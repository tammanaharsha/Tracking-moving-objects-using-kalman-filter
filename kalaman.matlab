clear all;
close all;
clc
 %% Read video into MATLAB using aviread
 video = mmreader('r2.avi');
 nframes = length(video);
 a=read(video,1);
 % Calculate the background image by averaging the first 5 images
 temp = zeros(size(a));
 [M,N] = size(temp(:,:,1));
 vidHeight = video.Height;
 vidWidth = video.Width;
 mov(1:nframes) = ...
 struct('cdata', zeros(vidHeight, vidWidth, 3, 'uint8'),...
 'colormap', []); 
for i = 1:10
 mov(i).cdata=read(video,i);
 %imshow(mov(i).cdata);
 temp = double(mov(i).cdata) + temp;
end
imbkg = temp/10;
 % Initialization for Kalman Filtering
 centroidx = zeros(nframes,1);
 centroidy = zeros(nframes,1);
 predicted = zeros(nframes,4);
 actual = zeros(nframes,4);
 % % Initialize the Kalman filter parameters
 % R - measurement noise,
 % H - transform from measure to state
 % Q - system noise,
 % P - the status covarince matrix
 % A - state transform matrix
 R=[[0.2845,0.0045]',[0.0045,0.0455]'];
 H=[[1,0]',[0,1]',[0,0]',[0,0]'];
 Q=0.01*eye(4);
 P = 100*eye(4);
 dt=1;
 A=[[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];
 % loop over all image frames in the video
 kfinit = 0;
 th = 38;
 for i = 1:nframes
 %mov(i).cdata=read(video,i);
 imshow(mov(i).cdata);
 %hold on
 imcurrent = double(mov(i).cdata);
% Calculate the difference image to extract pixels with more than 40(threshold) change
diffimg = zeros(M,N);
diffimg = (abs(imcurrent(:,:,1)-imbkg(:,:,1))>th) | (abs(imcurrent(:,:,2)-imbkg(:,:,
2))>th) | (abs(imcurrent(:,:,3)-imbkg(:,:,3))>th);
% Label the image and mark
labelimg = bwlabel(diffimg,4);
markimg = regionprops(labelimg,('basic'));
[MM,NN] = size(markimg);
% Do bubble sort (large to small) on regions in case there are more than 1
% The largest region is the object (1st one)
 for nn = 1:MM
 if markimg(nn).Area > markimg(1).Area
 tmp = markimg(1); 
 markimg(1)= markimg(nn);
 markimg(nn)= tmp;
 end
 end
 % Get the upper-left corner, the measurement centroid and bounding window size
 bb = markimg(1).BoundingBox;
 xcorner = bb(1);
 ycorner = bb(2);
 xwidth = bb(3);
 ywidth = bb(4);
 cc = markimg(1).Centroid;
 centroidx(i)= cc(1);
 centroidy(i)= cc(2);
% Plot the rectangle of background subtraction algorithm -- blue
hold on
rectangle('Position',[xcorner ycorner xwidth ywidth],'EdgeColor','b');
hold on
plot(centroidx(i),centroidy(i), 'bx');
% Kalman window size
kalmanx = centroidx(i)- xcorner;
kalmany = centroidy(i)- ycorner;
if kfinit == 0
 % Initialize the predicted centroid and volocity
 predicted =[centroidx(i),centroidy(i),0,0]' ;
else
 % Use the former state to predict the new centroid and volocity
 predicted = A*actual(i-1,:)';
end
kfinit = 1;
Ppre = A*P*A' + Q;
K = Ppre*H'/(H*Ppre*H'+R);
actual(i,:) = (predicted + K*([centroidx(i),centroidy(i)]' - H*predicted))';
P = (eye(4)-K*H)*Ppre;
% Plot the tracking rectangle after Kalman filtering -- red
hold on
rectangle('Position',[(actual(i,1)-kalmanx) (actual(i,2)-kalmany) xwidth
ywidth],'EdgeColor','r','LineWidth',1.5);
hold on
plot(actual(i,1),actual(i,2), 'rx','LineWidth',1.5);
drawnow;
