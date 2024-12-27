% MATLAB Script for the manuscript titled:
% Simplified Photogrammetry procedure for quantitative on-and-off surface flow topology studies over 3D surfaces: application to a Gaussian bump geometry 
% Created by Igal Gluzman (last updated on:12/27/2024)

%-----------------------------------------------------------------------------------
%References:  
% Gluzman, I., Gray, P., Corke, T.C. and Thomas, F.O., 2022. 
% A simplified photogrammetry procedure for
% quantitative on-and-off surface flow topology
% studies over 3D surfaces: application to a
% Gaussian bump geometry. Experiments in Fluids, XX(X), p.XXX.

% Before final publication of a paper using this script, please check Experiments in Fluids
% journal website for an up-to-date reference.
%-------------------------------------------------------------------------------------
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
%------------------------------------------------------------------------------------
clear 
% plot settings:
    whitebg('w') %create a figure with a white color scheme
    %set(0,'DefaultAxesColorOrder',[0 0 0])
    set(0,'DefaultAxesLineStyleOrder','-|--|:|-.')
    set(0,'DefaultLineLineWidth',2)
    set(0,'defaultaxeslinewidth',0.5)
    set(0,'defaultpatchlinewidth',0.5)
    set(0,'DefaultAxesFontSize',15)
    set(0,'DefaultLineMarkerSize',5)
    set(0,'DefaultTextInterpreter','tex')

close all; clc
%------------------------------------------------------------------------------------
flag_save=0; flag_Gifsave=0; %Flags to save figures and GIFs.if 1-save; if 0-don't save

%Guidelines:
% Load (at least 3) images of CALIBRATION CHECKERBOARD and defined the square Size in mm.
% The images should be from  different locations and angles of same CHECKERBOARD 
% See link for more details on Measuring Planar Objects with a Calibrated Camera
% https://www.mathworks.com/help/vision/ug/measuring-planar-objects-with-a-calibrated-camera.html

%Settings:

%1-select Folder name with calibration/experiment images

 F_data='bump72_Mp5_adjusted';squareSize =7.76; imval=3;%imval=3;%61.68 mm/8 downstream of bump
  
%2-select number of images (first N images in the F_data Folder)
 
%3-set CHECKERBOARD square size in mm.


files_list=dir( [F_data '\*.JPG']); 
numImages =length(files_list); %should be at least 3. %'bump36_Mp3_07142021'
filenamevec=cell(1,numImages); %Preallocate Memory for Cell Array
files = cell(1, numImages);
for i = 1:numImages 
    files{i} = [F_data '/' fullfile(files_list(i).name)];
    filenamevec{i}=[files_list(i).name];
end

%%
%4-Create folder to save the data:
foldername=['Plots_'  F_data]; 
mkdir(foldername)

%5-Display all calibration images and save

figure(31);clf
magnification = 10;
out = imtile( files);
imshow(out, 'InitialMagnification', magnification);
title('Calibration Images');
if flag_save==1
    exportgraphics(gca,[foldername '/Calibration Images.png'],'Resolution',600)
end
clear out
clc
%------------------------------------------------------------------------------------
%% step 1:Camera Calibration with checkerboard  (Estimate Camera Parameters)

% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).

worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
I = imread(files{1});% use one of the calibration images
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
                                     'ImageSize', imageSize);
clear I
%% Evaluate calibration accuracy.
figure(32); 
    showReprojectionErrors(cameraParams);
    title('Reprojection Errors');
    if flag_save==1
     exportgraphics(gca,[foldername '/ReprojectionErrors.png'],'Resolution',600)
    end
    
figure(33); clf
    showExtrinsics(cameraParams, 'PatternCentric');
    if flag_save==1
        exportgraphics(gca,[foldername '/PatternCentric.png'],'Resolution',600)
    end

%%  Selected image to proceed with
val=imval;
imOrig = imread(files{val});

figure(40); 
imshow(imOrig, 'InitialMagnification', magnification);
title('Input Image');
if flag_save==1
    exportgraphics(gca,[foldername '/imOrig_' filenamevec{val}(1:end-4) '.png'],'Resolution',600)
end

%%% Undistort the Image
% Use the cameraParameters object to remove lens distortion from the image. This is necessary for accurate measurement.

% Since the lens introduced little distortion, use 'full' output view to illustrate that
% the image was undistored. If we used the default 'same' option, it would be difficult
% to notice any difference when compared to the original image. Notice the small black borders.
%------MATLAB 2024a updated version:
ver=version;inds = strfind(ver,'R20');
if  str2num(ver(inds+1:inds+4))<2024
[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');

else

% https://www.mathworks.com/help/vision/ref/undistortimage.html
[im, intrinsics] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
newOrigin = cameraParams.Intrinsics.PrincipalPoint - intrinsics.PrincipalPoint
end
%---------------------------------------------------------
figure(40); clf
imshow(im, 'InitialMagnification', magnification); axis on
title('Undistorted Image');
if flag_save==1
    exportgraphics(gca,[foldername '/imUndist_' filenamevec{val}(1:end-4) '.png'],'Resolution',600)
end

%%%
 
% Detect the checkerboard.
%Find reference object (checkerboard) in new image (for the selected specific camera location).
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Compensate for image coordinate system shift:
% Adjust the imagePoints so that they are expressed in the coordinate system
% used in the original image, before it was undistorted.  This adjustment
% makes it compatible with the cameraParameters object computed for the original image.
imagePointsAdj = imagePoints+newOrigin; % adds newOrigin to every row of imagePoints

% Compute rotation and translation of the camera.
[R, tr] = extrinsics(imagePointsAdj, worldPoints, cameraParams);

%Compute camera pose that took imOrig:
% https://www.mathworks.com/help/vision/ref/extrinsics.html
[orientation_imOrig, location_imOrig] = extrinsicsToCameraPose(R,tr);
 
%%
sk=(boardSize(2)-1)*(boardSize(1)-1)-(boardSize(1)-2);
a=norm( worldPoints(sk,:)-worldPoints(1,:))./norm( imagePoints(sk,:)-imagePoints(1,:));

worldPoints_adj = pointsToWorld(cameraParams, R, tr,imagePointsAdj); %as observed from the selected camera (thus we use imagePointsAdj)
 
% Compute the distance to the camera.
[cameraOrientation, cameraLocation] = extrinsicsToCameraPose(R, tr);

%------------------------------------------------------------------------------------
%% Step 2. Relate reference points between image and surface:

%Define reference point (at the base of the bamp) and validation point (to valide the mapping). 

close all
   
   
 if  strcmp(F_data,'bump72_Mp5_adjusted')
     if val==1
        % Define Reference point (at the base of the bamp:
        ref_0_px_1y=3054;%px
        ref_0_px_1x=2882; %px

        % Define  validation point of interest (on the  bamp surface):
        ref_pick_px(1,1) =2874;         % [pixels] x-location of 1st point upper left with horizontal dist_x  from it
        ref_pick_px(1,2) = 219;          % [pixels] y-location of 1st point upper left with vertical dist_y  from it
     end
     if val==2% At high camera angle
        % Define Reference point (at the base of the bamp:
        ref_0_px_1y=2745;  %px
        ref_0_px_1x=2190; %px

        % Define  validation point of interest (on the  bamp surface):
        ref_pick_px(1,1) =4070;         % [pixels] x-location of 1st point upper left with horizontal dist_x  from it
        ref_pick_px(1,2) = 376;          % [pixels] y-location of 1st point upper left with vertical dist_y  from it
          end

     if val==3% At high camera angle
        % Define Reference point (at the base of the bamp:
        ref_0_px_1y=2698; %px
        ref_0_px_1x=3836; %px

        % Define  validation point of interest (on the  bamp surface):
        ref_pick_px(1,1) =2200;         % [pixels] x-location of 1st point upper left with horizontal dist_x  from it
        ref_pick_px(1,2) = 382;         % [pixels] y-location of 1st point upper left with vertical dist_y  from it
     end
 end
 
% Reference point calculations: 
ref_0=[ref_0_px_1x ref_0_px_1y];
ref_0_Adj=ref_0+ newOrigin;
% Get the world coordinates of the points: 
worldPoints_ref_0_Adj= pointsToWorld(cameraParams, R, tr, ref_0_Adj);  % Get the world coordinates of the ref_0_Adj:


%Validation point calculations
dist_x=0;   
dist_y=0; 
ref_pick_px(2,1) = ref_pick_px(1,1)+dist_x;            % [pixels] x-location of 2nd point lower right
ref_pick_px(2,2) = ref_pick_px(1,2)+dist_y;            % [pixels] y-location of 2nd point lower right
ref_pick=[(ref_pick_px(1,1)+ref_pick_px(2,1))/2  (ref_pick_px(1,2)+ref_pick_px(2,2))/2 ];
ref_pickAdj=ref_pick+ newOrigin;
% Get the world coordinates of the points: 
worldPoints_ref_pick= pointsToWorld(cameraParams, R, tr, ref_pickAdj);% Get the world coordinates of interest point: 


X_IP=worldPoints_ref_pick(1);%Bump apex orange dot
X_ref=worldPoints_ref_0_Adj(1);%Bump base orange dot

Y_IP=worldPoints_ref_pick(2);%Bump apex orange dot-red circle
Y_ref=worldPoints_ref_0_Adj(2);%Bump base orange dot-blue circle


figure(41);clf
imshow(im);hold on
plot(imagePoints(1,1),imagePoints(1,2),'xg','markersize',10,'linewidth',3);%Origin chessboard.
plot(ref_0(1) ,ref_0(2) ,'om','markersize',10,'linewidth',3);              %Reference point to model  which we can compute the distance from origin.
plot(ref_pick(1),ref_pick(2),'or','markersize',10,'linewidth',3);          %Point of interest.
xlabel('pixels')
ylabel('pixels')
legend(['Origin: x, mm=' num2str(worldPoints_adj(1,1)) '; y, mm=' num2str(worldPoints_adj(1,2))],...
    ['Ref 0: x, mm=' num2str(X_ref) '; y, mm=' num2str(Y_ref)],...
    ['Ref pick: x, mm=' num2str(X_IP) '; y, mm=' num2str(Y_IP)]...
    ,'location','southout');
title('Undistored Image')
 
axis on
  
norm_Xfix=norm([X_ref Y_ref] -[X_IP Y_IP]); 
%%
disp(['Calib: |X_ref_0-X_ref_pick|=' num2str(norm_Xfix) 'mm'])
if flag_save==1
    exportgraphics(gca,[foldername '/imOrigwithRefPoints' filenamevec{val}(1:end-4) '.png'],'Resolution',600)
end

%------------------------------------------------------------------------------------
%% Step 3. Map the word points of the oriented surface to image points.

% Surface function of bump geometry
clear Xmesh Ymesh X Y Z L

%%Gaussian surface
    inchtomm=25.4;
    L=36*inchtomm; %inch, 914.4 mm-Spanwise length
    h=0.085*L;%3.06 inch-validated by physical measurment with patrick
    y_0=0.06*L;
    x_0=0.195*L;
    Gridsize=1000;%Grid size
    Xmesh=linspace(0,40*inchtomm,Gridsize);%1:Lx; the section length is 40 inches.
    Ymesh=linspace(0,L,Gridsize);
    [X,Y] = meshgrid(Xmesh,Ymesh);
    shift_x=40*inchtomm/2;
    shift_y=L/2;
    X_shift=X-shift_x;
    Y_shift=Y-shift_y;
    Z=0.5*h*(1+erf((0.5*L-2*y_0-abs(Y_shift))/y_0)).*exp(-(X_shift/x_0).^2);
    % err=0.05*9;
    err=0.05*10;%updated for Matlab2024
    
% figure(50);clf
% s = surf(X,Y,Z,'FaceAlpha',0.5);hold on
% s.EdgeColor = 'none';
% xlabel('X, mm'); ylabel('Y, mm'); zlabel('-Z, mm')
% title('Model Geomtry')
% % view([0 0])
% daspect([1 1 1])
%        cmp =  colormap; 
%        cmpR = flipud(cmp); 
%     if cmpR(end)>0.5
%       colormap(cmpR); 
%     end
% if flag_save==1
%     exportgraphics(gca,[foldername '/Bamp' '.png'],'Resolution',600)
% end
  % c=colorbar; c.Label.String = '-Z, mm'
%Shift axes: Put surface axis on center of Y and reverse X direction for downstream configuration:
     X_shift=(X-X(end,end)); 
     Y_shift=(Y-Y(end,end)/2); 

   

figure(50);clf
s = surf(X_shift,Y_shift,Z,'FaceAlpha',0.5);hold on
s.EdgeColor = 'none';
xlabel('X shift, mm'); ylabel('Y shift, mm'); zlabel('Z, mm')
title('Model Geomtry')
% view([0 0])
daspect([1 1 1])
cmp =  colormap; 
cmpR = flipud(cmp); 
if cmpR(end)>0.5
    colormap(cmpR); 
end
c=colorbar; c.Label.String = '-Z, mm'

if flag_save==1
    exportgraphics(gca,[foldername '/Bamp' '.png'],'Resolution',600)
end
%% Fix perspective distortion due to bump height and flat chessboard assumption:

% Adjust calibration of surface:
 
 
alpha=atand((Y_IP-Y_ref)/(X_IP-X_ref) );  %  account for camera view angle for spanwsie correction.
Z_P=h; %Bump apex height.

 
A_x=-1;%if downstream of the hamp (goeas against chessboard direction which orineted to free stream.
X_P=X_ref+A_x*shift_x;%Surface real apex (not from image)
Z_ref= Z_P*abs(X_IP-X_ref)/abs(X_IP-X_P)

if Y_ref>Y_IP
A_y=1;
else
  A_y=-1;  
end
Y_P=Y_ref-shift_x*tand(alpha) ; %Surface real apex  (not from image)--angle correction is applied due to camera view angle.  
Z_ref2= Z_P*abs(Y_IP-Y_ref)/abs(Y_IP-Y_P); % Z_ref2 should be same as Z_ref
 

 %Stretching coefficient:
    C_stretch_apex=1./(1-h/Z_ref); %only for ref_pick reference point.
    C_stretch=1./(1-Z/Z_ref); % for any point on the CAD.

     
 %Stretching coefficient using camera location
    X_cam=cameraLocation(1);
    Y_cam=cameraLocation(2);
    Z_cam=cameraLocation(3);
    C_stretch_cam_apex=1./(1-h/abs(Z_cam));
 
 

%% Set reference point on the surface: (Input data from experiment)

 
%True reference points as should be on CAD model:
surf_ref_0_mm_1x_true=0;%(L/2)*inchtomm;%mm
surf_ref_0_mm_1y_true=1*inchtomm; %mm

ref_0_true=[surf_ref_0_mm_1x_true surf_ref_0_mm_1y_true];% True   ref_0     reference point on CAD axis.
ref_pick_true=[A_x*shift_x  surf_ref_0_mm_1y_true];      % True   ref_pick  reference point on CAD axis.


%False  ref_pick point  from image  on CAD model:

surf_ref_pick_image(1)=ref_0_true(1)+A_x*abs(X_IP-X_ref); % mm 
surf_ref_pick_image(2)=( ref_0_true(2)-shift_x*tand(alpha) )+A_y*abs(Y_IP-Y_ref); %mm-Account for view angle.

%Corrected by Streching coeffiencet ref_pick point  from image  on CAD model:


surf_ref_pick_image_C(1)=ref_0_true(1)+A_x*abs(X_IP-X_ref)/C_stretch_apex; % mm 
surf_ref_pick_image_C(2)=( ref_0_true(2)-shift_x*tand(alpha) )+A_y*abs(Y_IP-Y_ref)/C_stretch_apex; %mm

%Option 2: correction by camera strech coeff:
surf_ref_pick_image_C2(1)=-X_ref+X_cam+A_x*abs(X_IP-X_cam)/C_stretch_cam_apex; % mm 

figure(52);clf
ax1 =subplot(2,1,2);
s = surf(X_shift,Y_shift,Z,'FaceAlpha',0.5);hold on
s.EdgeColor = 'none';
px=xlabel('X, mm'); 
py=ylabel('Y, mm'); 
pz=zlabel('Z, mm');
cmp = flipud(colormap);colormap(cmp);
%plot real points:
pl=yline(0,'r');
p1=plot(ref_0_true(1),ref_0_true(2) ,'xb','markersize',10,'linewidth',3); % ref_0 true reference point on bump coordinates.
p3=plot3(ref_pick_true(1),ref_pick_true(2),h,'xk');%apex-true reference point of apex shited one inch on bump coordinates
p3=plot3(A_x*shift_x,0,h,'*k');% true apex location of the bump

%plot ref_point_pick points from image before stretching correction:

p2=plot3(surf_ref_pick_image(1),surf_ref_pick_image(2),0,'*r','markersize',10,'linewidth',3); %reference point to model  which we can compute the distance from origin.
%%%%

%plot ref_point_pick from image after stretching correction:
p21=plot3(surf_ref_pick_image_C(1),surf_ref_pick_image_C(2),0,'og','markersize',10,'linewidth',3); %reference point to model  which we can compute the distance from origin.


plotObjs = [s,p1,p21,p2,p3,px,py,pz,pl];
 %daspect ([1 1 1]);
    view(ax1,2);  
    ylim([-40 40])
%   ylim([25 30])
ax2=subplot(2,1,1);
    copyobj(plotObjs,ax2);%Copy plot objects to other 2 subplots
    daspect ([1 1 1]);
    view(ax2,0,0);  
legend(' ','apex','interest','stretched interest','location','north','Orientation','horizontal') 

if flag_save==1
    legend off
    exportgraphics(gcf,[foldername '/Bampstretched' '.png'],'Resolution',600)
end
% get Z coordinate of the surface at surf_ref_pick:
 
clear indx 1 indx2 Z_ref_pick
indx1_fixed=find(abs(Y_shift(:,1)-surf_ref_pick_image(2))<err); indx1_fixed=indx1_fixed(1);
 
indx2_fixed=find(abs(X_shift(1,:)-surf_ref_pick_image(1))<err*1.5); indx2_fixed=indx2_fixed(1);
 
Z_ref_pick=Z(indx1_fixed,indx2_fixed);
surf_ref_pick_image(3)=Z_ref_pick;
 
%% plot the surface with camera:



%Adjust axis directions 
%Downstream of bump
ax=1; 
ay=-1;
az=-1;

surf_to_ext(1)=X_ref+ref_0_true(1);
surf_to_ext(2)=Y_ref+ref_0_true(2);
 
%Word points of surface in chekerboard coordinates:
WPsurf_x_shift=ax*(X_shift)+surf_to_ext(1);% IG test
WPsurf_y_shift=ay*Y_shift+surf_to_ext(2);
WPsurf_z_shift=az*Z;


%False  ref_pick point  from image  on CAD model in checkerboard coordinates:

ref_pick_image(1)=X_ref+ref_0_true(1)+A_x*abs(X_IP-X_ref); % mm 
ref_pick_image(2)=Y_ref+( -shift_x*tand(alpha) )+A_y*abs(Y_IP-Y_ref); %mm


%Corrected by Stretching coefficient ref_pick point  from image  on CAD model:


ref_pick_image_C(1)= X_ref+ref_0_true(1)+A_x*abs(X_IP-X_ref)/C_stretch_apex; % mm 
ref_pick_image_C(2)=Y_ref+( -shift_x*tand(alpha) )+A_y*abs(Y_IP-Y_ref)/C_stretch_apex; %mm %account for view angle.

for mm=[1 3]
    figure(50+mm); clf 
    % ax1=subplot(2,1,1);
    ex=showExtrinsics(cameraParams, 'PatternCentric'); hold on
     
    s = surf(WPsurf_x_shift,WPsurf_y_shift,WPsurf_z_shift,'FaceAlpha',0.5);hold on
    s.EdgeColor = 'none';
    
    %------------------------------
    
    %plot real points:
    pl=yline(surf_to_ext(2),'r');
    p1=plot(X_ref,Y_ref,'xb','markersize',10,'linewidth',3); % ref_0 true reference point on bump coordinates.
    p3=plot3(X_P,Y_ref,0,'xk');%apex-true reference point of apex shited one inch on bump coordinates
    p3=plot3(X_P,surf_to_ext(2),-h,'*k');% true apex location of the bump
    
    % view(90,0)
    
     
    
    
    %plot ref_point_pick points from image before streching correction:
    
    p2=plot3(ref_pick_image(1),ref_pick_image(2),0,'*r','markersize',10,'linewidth',3); %reference point to model  which we can compute the distance from origin.
    %%%%
    
    %plot ref_point_pick from image after stretching correction:
    p21=plot3(ref_pick_image_C(1),ref_pick_image_C(2),0,'og','markersize',10,'linewidth',3); %reference point to model  which we can compute the distance from origin.
    
    
    plot3(cameraLocation(1) ,cameraLocation(2),cameraLocation(3),'.','markersize',10,'linewidth',3); %reference point to model  which we can compute the distance from origin.
    
    
     view(45,30)
    if mm==3
        view(0,90)%top view xy-plane
    end
    % view(90,0)%side view yz-plane
    % view(0,0)%side view xz-plane
    % ylim([0 140])
      
     
    if flag_save==1
        exportgraphics(gcf,[foldername '/BampCamera' '.png'],'Resolution',600)
    end
    
    if flag_save==1
        exportgraphics(gcf,[foldername '/BampCamera4530' '.png'],'Resolution',600)
    end
end
%% Step 4. Interpolated surface in the pixel domain over the pixels of the image. 

% %map the word points of the joint (plot with surface and camera) to image point:
 
clear worldPoints_surf_deform worldPoints_surf_deform_ext
%Get imagepoints of surface:
worldPoints_surf_deform=[((WPsurf_x_shift(:))).*C_stretch(:)  ((WPsurf_y_shift(:))).*C_stretch(:)   WPsurf_z_shift(:)*0]; %test: I dont use bump heihft, but use streching to account for it


for jj=[1 2]
    if jj==1
        gamma=0
    if  alpha<0
        AngC=-(WPsurf_x_shift-X_ref)*tand(alpha);%start correction after this.
    else
        AngC=(WPsurf_x_shift-X_ref)*tand(alpha);%start correction after this.
    end

    elseif jj==2
        %distance between apex and shifter pick refernece point in pixels:
    dist_pxl=norm([mean(xx(indxzzmax)) mean(yy(indxzzmax))]-[ref_pick(1) ref_pick(2)]);
    
    %angle to rotate the CAD model:
     gamma=atand(  (ref_pick(2)-mean(yy(indxzzmax)))/(ref_pick(1)-mean(xx(indxzzmax)))  )
   
     
     %use a   ' mm/pixel'] to tranfer to distance in mm:
    dist_mm=dist_pxl*a;%distance from the center 

     DY=inchtomm/C_stretch_apex-dist_mm ; %should be 1 inch shift. the remaining distance is the DY
    %option 1: works better
     beta=atand(DY/shift_x);%V11

    

    
    if  beta<0
        AngC2=-(WPsurf_x_shift-X_ref)*tand(beta);
    else
          AngC2=(WPsurf_x_shift-X_ref)*tand(beta);
    end


    if  alpha>0  &&   beta<0
        AngC=AngC+AngC2;
    elseif  (alpha<0  &&   beta>0) || (alpha>0  &&   beta>0)
         AngC=AngC-AngC2;
    else 
         AngC=AngC-AngC2;
    end


    end

    worldPoints_surf_deform=[((WPsurf_x_shift(:))).*C_stretch(:)  ((WPsurf_y_shift(:))+AngC(:) ).*C_stretch(:)  WPsurf_z_shift(:)*0]; 
    
    worldPoints_surf_deform_ext(:,1)=worldPoints_surf_deform(:,1);
    worldPoints_surf_deform_ext(:,2)=worldPoints_surf_deform(:,2);
    worldPoints_surf_deform_ext(:,3)=worldPoints_surf_deform(:,3);
    
    %  Find a reference object in the new image.
    [imagePoints,boardSize] = detectCheckerboardPoints(im);
    %Compute new extrinsics.
    [RNew,trNEw] = extrinsics(imagePoints,worldPoints,cameraParams);
    [orientation2, locationNew2] = extrinsicsToCameraPose(RNew,trNEw);
    
     
    
    projectedPoints_Surf =worldToImage(cameraParams,RNew,trNEw,worldPoints_surf_deform_ext);
    
    
    figure(61);clf
    imshow(im, 'InitialMagnification', magnification*3);
    hold on
    title('Undistored Image')
    xx=projectedPoints_Surf(:,1);
    yy=projectedPoints_Surf(:,2);
    zz=-WPsurf_z_shift(:);
    c = 1:numel(-WPsurf_z_shift(:));      %# colors
    h1 = surface([xx(:), xx(:)], [yy(:), yy(:)], [zz(:), zz(:)], 'EdgeColor','flat', 'FaceColor','none','EdgeAlpha',.4);
    disp(h1);
    % axis([-500 6000 -1000 4000])
    indxzzmax=find(zz==max(max(zz)));
    plot(imagePoints(1,1),imagePoints(1,2),'xg')
    plot3(xx(indxzzmax),yy(indxzzmax),zz(indxzzmax),'.c')
    plot3(mean(xx(indxzzmax)),mean(yy(indxzzmax)),mean(zz(indxzzmax)),'xy')
    %Tru point:
    pick_true_x=(mean(xx(indxzzmax))+cosd(gamma)*inchtomm/C_stretch_apex/a);
    plot(ref_0(1),ref_0(2),'om','markersize',10,'linewidth',3);
    pick_true_y=(mean(yy(indxzzmax))+sind(gamma)*inchtomm/C_stretch_apex/a);
    plot3(pick_true_x,pick_true_y,mean(zz(indxzzmax)),'og')
    plot3(ref_pick(1),ref_pick(2),h,'or','markersize',10,'linewidth',3);%Point of intrest.
    % legend(' ','origin','ref','apex region','apex','ref pick true','ref pick')
    axis on
    if flag_save==1
        exportgraphics(gcf,[foldername '/ImSurface' '.png'],'Resolution',600)
    end
    
 end
     
    % ylim( mean(yy(indxzzmax))+[-200 200])
    % xlim( mean(xx(indxzzmax))+[-500 500])
     
    if flag_save==1
        exportgraphics(gcf,[foldername '/ImSurfaceZoomed' '.png'],'Resolution',600)
    end

%%  Interpolating Scattered Data of the word points of surface into undistored image domain:
%https://www.mathworks.com/help/matlab/math/interpolating-scattered-data.html

clear xi yi zi Cx_inv CY_inv
[xi,yi] = meshgrid(1:size(im,2), 1:size(im,1));

zi = griddata(xx,yy,zz,xi,yi);

Cx_inv= griddata(xx,yy,C_stretch(:),xi,yi);%stretching coeffeinet to restore the original shap of the hamp
Cy_inv= griddata(xx,yy,C_stretch(:),xi,yi);

AngCorrection=griddata(xx,yy,AngC(:),xi,yi);%interpolate angle correction for extended grid that match image domain size.

% remove nan values from zi:
 TF = isnan(zi);
 zi_0=zi;
 zi_0(TF==1)=0;
 
 Cx_inv(TF==1)=1;
 Cy_inv(TF==1)=1;

%% Replace intensity value of image with height value of surface:
indxzimax=find(zi==max(max(zi)));
figure(63);clf
sww=surf(xi,yi,zi_0,'FaceAlpha',0.5);hold on
sww.EdgeColor = 'none';
view(2);
axxx = gca;
hold on
indxzi0max=find(zi_0==max(max(zi_0)));
plot3(xi(indxzi0max),yi(indxzi0max),zi_0(indxzimax),'oy')
axis ij
xlabel('pixels')
ylabel('pixels')

plot(imagePoints(1,1),imagePoints(1,2),'xg','markersize',10,'linewidth',3);%Origin chessboard.
plot(ref_0(1) ,ref_0(2) ,'om','markersize',10,'linewidth',3); %reference point to model  which we can compute the distance from origin.
plot(ref_pick(1),ref_pick(2),'or','markersize',10,'linewidth',3);%Point of intrest.

 view(2)
 if flag_save==1
    exportgraphics(gcf,[foldername '/SurfaceInterpToIm' '.png'],'Resolution',600)
 end
%----------------------------------------------------------------------------------------------------------
 %% Step 5. Warp the oil image on the interpolated surface in a mixed ”world-point pixel domain” x − y − Z.
close all;
figure(71);clf
hwarp=warp(xi,yi,zi_0,im);%imOrig
hold on
%  view(3)
  view([80 50])
disp(hwarp)
axis square
plot3(xi(indxzimax),yi(indxzimax),zi(indxzimax),'xy')

xlabel('x, pixel')
ylabel('y, pixel')
zlabel('z, mm')
% zlim([40 90])
% ylim([0 900])
if flag_save==1
    exportgraphics(gcf,[foldername '/WarpImSurface' '.png'],'Resolution',600)
end
%% Project image on surface using hwarp.CData:

 
imAdj=[xi(:) yi(:)]; % No need to add +newOrigin because we use RNew, trNEw for  the camera of Img_Orig

% Get the world coordinates of the points: 

worldPoints_im= pointsToWorld(cameraParams, RNew, trNEw, imAdj);% Get the world coordinates of interest point: 

  imAdjX= reshape(worldPoints_im(:,1),size(zi_0));
  imAdjY=  reshape(worldPoints_im(:,2),size(zi_0));
  Cx_imAdjX=reshape(Cx_inv(:),size(zi_0));
  Cy_imAdjY=reshape(Cy_inv(:),size(zi_0));

clear xx yy zz Cx_inv CY_inv xi yi zi imAdj worldPoints_im
%--------------------------------------------------------------------------------------------  
%% Step 6. transform the warped bump from mixed x − y − Z domain to world point domain X − Y − Z.

figure(72);clf
 s = surf(imAdjX,imAdjY,zi_0,hwarp.CData); hold on
 s.EdgeColor = 'none';
%  daspect([1 1 1])
 xlabel('x, mm')
 ylabel('y, mm')
 zlabel('z, mm')
 view([80 50])
 title('before stretching')
  if flag_save==1
   exportgraphics(gcf,[foldername '/ExtrinWarpImSurface' '.png'],'Resolution',600)
 end

 
figure(73);clf
subplot(2,1,1)
 s = surf(imAdjX,imAdjY,zi_0,hwarp.CData); hold on
 s.EdgeColor = 'none';
 daspect([1 1 1])
 xlabel('x,mm')
 ylabel('y, mm ')
 zlabel('z, mm ')
 view([0 0])
 title('before stretching')

%-----------------------------------------------------------------------------
%% Step 7. Apply mapping correction on the interpolated surface in world points.

figure(74);clf
subplot(2,1,2)
 s_im = surf((imAdjX)./Cx_imAdjX,(imAdjY)./Cy_imAdjY,zi_0,hwarp.CData); disp(s_im)
 s.EdgeColor = 'none';
 daspect([1 1 1])
 xlabel('x,mm')
 ylabel('y, mm ')
 zlabel('z, mm ')
view([0 0])
 title('after stretching')
 
 
 if flag_save==1
    exportgraphics(gcf,[foldername '/ExtrinWarpImSurfaceStretch' '.png'],'Resolution',600)
 end

%% Step 8. Plot the interpolated surface in world points with the original CAD model in the checkerboard coordinate system. 

% Match the warped image surface with bump geometry:
 

f1=figure(75);clf
showExtrinsics(cameraParams, 'PatternCentric'); hold on
%Original surface without any stretching:
s = surf(ax*X_shift+surf_to_ext(1),ay*Y_shift+surf_to_ext(2),az*Z,'FaceAlpha',0.5);
s.EdgeColor = 'none';
 
%After image is projected to real-word we need to fix it by stretching coefficient  
%Slightly elevated the surface of the image to show clearly the oil pattern.

undeformX=imAdjX./Cx_imAdjX ;
undeformY=(imAdjY)./Cy_imAdjY-AngCorrection;%V8

s_im = surf(undeformX,undeformY,-zi_0-0.1,hwarp.CData);  s_im.EdgeColor = 'none';hold on

p3=plot3(A_x*shift_x+surf_to_ext(1),0+surf_to_ext(2),-h,'xy'); 
pl=yline(0+surf_to_ext(2),'r');
p1=plot(ax*ref_0_true(1)+surf_to_ext(1),ay*ref_0_true(2)+surf_to_ext(2) ,'om','markersize',10,'linewidth',3); %reference point to model  which we can compute the distance from origin.


p233333=plot3((ax*surf_ref_pick_image_C(1)+surf_to_ext(1)),(ay*surf_ref_pick_image_C(2)+surf_to_ext(2)),-h,'or','markersize',10,'linewidth',3); %reference point to model  which we can compute the distance from origin.

xline(A_x*(shift_x-126)+surf_to_ext(1),'g');%correcred
zlim([-100 0])

 
px=xlabel('x,mm');
py=ylabel('y, mm ');
pz=zlabel('z, mm ');

plotObjs110 = [s,s_im,p1,p2,p3,pl];

title(' ')
 
%  view([50,15])
%  view(2)
%  view([0,0])% xz- Side view
%  view([180,0])% xz- Side view other side
   view([90,90])%yx-Top view
%  view([90,0])% yz view-rare: from upstream
%  view([270,0])% yz view-front
%  view([51,26])%Isometric downstream view
 if flag_save==1
    exportgraphics(gcf,[foldername '/ExtrinWarpImSurfaceAll2' '.png'],'Resolution',600)
    savefig(f1,[foldername '/ExtrinWarpImSurfaceAll2' '.fig'])
 end
view([45,30])
 if flag_save==1
    exportgraphics(gcf,[foldername '/ExtrinWarpImSurfaceAll2_4530' '.png'],'Resolution',600)
    savefig(f1,[foldername '/ExtrinWarpImSurfaceAll2_4530' '.fig'])
 end
 %% animate figure:
if flag_Gifsave==1

azi = 0;
el = 90;
view([azi,el])
degStep = 5;
detlaT = 0.1;
fCount = 71;
f = getframe(gcf);
[imf,map] = rgb2ind(f.cdata,256,'nodither');
imf(1,1,1,fCount) = 0;
k = 1;
% spin 45°
for i = 0:-degStep:-45
  azi = i;
  disp([azi,el]);
  f = getframe(gcf);
  imf(:,:,1,k) = rgb2ind(f.cdata,map,'nodither');
  k = k + 1;
end
% tilt down
for i = 90:-degStep:15
  el = i;
  view([azi,el])
  f = getframe(gcf);
  imf(:,:,1,k) = rgb2ind(f.cdata,map,'nodither');
  k = k + 1;
end
% spin left
for i = azi:-degStep:-90
  azi = i;
  view([azi,el])
  f = getframe(gcf);
  imf(:,:,1,k) = rgb2ind(f.cdata,map,'nodither');
  k = k + 1;
end
% spin right
for i = azi:degStep:0
  azi = i;
  view([azi,el])
  f = getframe(gcf);
  imf(:,:,1,k) = rgb2ind(f.cdata,map,'nodither');
  k = k + 1;
end
% tilt up to original
for i = el:degStep:90
  el = i;
  view([azi,el])
  f = getframe(gcf);
  imf(:,:,1,k) = rgb2ind(f.cdata,map,'nodither');
  k = k + 1;
end

 imwrite(imf,map,[foldername '/ExtrinWarpImSurfaceAll3.gif'],'DelayTime',detlaT,'LoopCount',inf)
end
  
 %% shift all to bump coordinate system where axis origin is for merging with PIV data

xbs=A_x*(shift_x*0)+surf_to_ext(1)-shift_x;%x_bump_shift
ybs=0+surf_to_ext(2);%y_bump_shift
 
f2=figure(120);clf
showExtrinsics(cameraParams, 'PatternCentric'); hold on
s = surf(ax*X_shift+surf_to_ext(1)-xbs,ay*Y_shift+surf_to_ext(2)-ybs,az*Z,'FaceAlpha',0.5);
s.EdgeColor = 'none';
s_im = surf(undeformX-xbs,undeformY-ybs,-zi_0-0.1,hwarp.CData);  s_im.EdgeColor = 'none';hold on
p233333=plot3((ax*surf_ref_pick_image_C(1)+surf_to_ext(1))-xbs,(ay*surf_ref_pick_image_C(2)+surf_to_ext(2))-ybs,-h,'or','markersize',10,'linewidth',3); %reference point to model  which we can compute the distance from origin.
xline(A_x*(shift_x-126)+surf_to_ext(1)-xbs,'g');% correcred
zlim([-100 0])
daspect([1 1 0.1])
px=xlabel('x,mm');
py=ylabel('y, mm ');
pz=zlabel('z, mm ');

plotObjs110 = [s,s_im,p1,p2,p3,pl];
 set (gca,'Zdir','reverse')
 set (gca,'Ydir','reverse')
%  set (gca,'Zdir','reverse')
%  view([50,15])
%  view(2)
%  view([0,0])% xz- Side view
%  view([180,0])% xz- Side view other side
%   view([90,90])%yx-Top view
%  view([90,0])% yz view-rare: from upstream
%  view([270,0])% yz view-front
%  view([51,26])%Isometric downstream view

p3=plot3(A_x*shift_x+surf_to_ext(1)-xbs,0+surf_to_ext(2)-ybs,-h,'oy'); 
pl=yline(0+surf_to_ext(2)-ybs,'r');
p1=plot3(ax*ref_0_true(1)+surf_to_ext(1)-xbs,ay*ref_0_true(2)+surf_to_ext(2)-ybs,-0.1 ,'om','markersize',10,'linewidth',3); %reference point to model  which we can compute the distance from origin.


daspect([1 1 1])
 
 if flag_save==1
    exportgraphics(gcf,[foldername '/ExtrinWarpImSurfaceAll2shift' '.png'],'Resolution',600)
    savefig(f2,[foldername '/ExtrinWarpImSurfaceAll2shift' '.fig'])
 end

 
view([45,30])
 if flag_save==1
    exportgraphics(gcf,[foldername '/ExtrinWarpImSurfaceAll2shiftZoomed' '.png'],'Resolution',600)
    savefig(f2,[foldername '/ExtrinWarpImSurfaceAll2shiftZoomed' '.fig'])
 end
 
 
