function varargout = poseGt( action, varargin )
% Object pose annotations struct.
%
% phi describes an instance of an object, while the object model describes
% how phi is interpreted. poseGt allows for articulated object models
% organized in a tree structure. A model is composed of m parts (this first
% always being the root), where each part has the following fields:
%  prn    - part parent (or 0 if root)
%  joint  - [2x1] x and y joint location (relative to parent)
%  lks    - [3x1] 0/1 values indicating if given value is locked
%  wts    - [3x1] relative importances when computing distances
%  mus    - [3x1] means for generating random poses
%  sigs   - [3x1] sigs for generating random poses
% The lks, wts, mus and sigs are all of the form: [ang scl asp]. The
% exception is the root which has an x,y position and hence the lks, wts,
% mus and sigs are all of the form: [x y ang scl asp]. The joint is given
% in coordinates relative to each part's parent, in a coordinate system
% where the part's parent is a unit circle (the root has no joint). The
% units for ang are radians, while scl and asp are in log2 space (i.e.
% scl=-1 indicates a scale of 2^-1=.5). Root position is given in aboslute
% (pixel) coordinates, while for all parts scl and ang (but not asp) are
% givern relative to the part's parent.
%
% poseGt contains a number of utility functions, accessed using:
%  outputs = poseGt( 'action', inputs );
% The list of functions and help for each is given below. Also, help on
% individual subfunctions can be accessed by: "help poseGt>action".
%
%%% (1) Data structure for storing object part annotations.
% Create pose model (model is necessary for all other actions).
%   model = poseGt( 'createModel', type )
% Create single part for model (parent==0 implies root).
%   part = poseGt( 'createPart', parent, wts )
% Save object part annotations to text file.
%   model = poseGt( 'objSave', model, phis, fName )
% Load object part annotations from text file.
%   phis = poseGt( 'objLoad', model, fName )
%
%%% (2) Pose display support.
% Draw object with parameters phis on top of Is.
%   h = poseGt( 'draw', model, Is, phis, varargin )
% Draw results overlaid on ground truth, separated into quantiles.
%   h = poseGt( 'drawRes', model, Is, phisGt, phis, varargin )
% Generate toy images for testing each pose model.
%   [Is,phis] = poseGt( 'toyData', model, n, h, w, varargin )
%
%%% (3) Routines for getting/setting object position.
% Return [n x m x 5] absolute position for each object and each part.
%   bbs = poseGt( 'getBbs', model, phis, [ellipse] )
% Set location of part id using given absolute position.
%   phi = poseGt( 'setBbs', model, phi, id, bb )
% Return cumulative pose for each part.
%   [xs,ys,ang,scl,asp,HS] = poseGt( 'getPose', model, phis )
% Compute binary mask of pixels belonging to each object.
%   masks = poseGt( 'getMask', model, phis, h, w )
% Convert objs created by bbGt to phis.
%   phis = poseGt( 'objsToPose', objs )
%
%%% (4) Support for pose-indexed features.
% Generate random pose indexed features.
%   ftrData = poseGt( 'ftrsGen', model, varargin )
% Take subset of features according to fids.
%   [ftrData,fids1] = poseGt( 'ftrsSubset', model, ftrData, fids0 )
% Compute pose indexed features on Is given phis.
%   [ftrs,Vs] = poseGt( 'ftrsComp', model, phis, Is, ftrData, imgIds )
%
%%% (5) Routines for manipulating phis.
% Return N copies of the identity element.
%   phis = poseGt( 'identity', model, N )
% Generate N random phis according to the model.
%   phis = poseGt( 'random', model, N )
% Compose phis0 and phis1: phis = phis0 + phis1.
%   phis = poseGt( 'compose', model, phis0, phis1 )
% Compute inverse of phis0 so that phis0+phis1=phis1+phis0=identity.
%   phis = poseGt( 'inverse', model, phis0 )
% Compute phi that minimizes sum of distances to phis.
%   phi = poseGt( 'compPhiStar', model, phis );
% Compute diffs between phis0(i,:,t) and phis1(i,:) for each i and t.
%   del = poseGt( 'diff', phis0, phis1 )
% Compute distance between phis0(i,:,t) and phis1(i,:) for each i and t.
%   [ds,dsAll] = poseGt( 'dist', model, phis0, phis1 )
% Compute distance between phis0(i,:) and phis1(j,:) for each i and j.
%   D = poseGt( 'dist2', model, phis0, phis1 )
%
% USAGE
%  varargout = poseGt( action, varargin );
%
% INPUTS
%  action     - string specifying action
%  varargin   - depends on action, see above
%
% OUTPUTS
%  varargout  - depends on action, see above
%
% EXAMPLE
%
% See also poseGt>createModel, poseGt>createPart, poseGt>objSave,
% poseGt>objLoad, poseGt>draw,  poseGt>drawRes, poseGt>toyData,
% poseGt>getBbs, poseGt>setBbs, poseGt>getPose, poseGt>getMask,
% poseGt>objsToPose, poseGt>ftrsGen, poseGt>ftrsSubset, poseGt>ftrsComp,
% poseGt>identity, poseGt>random, poseGt>compose, poseGt>inverse,
% poseGt>compPhiStar, poseGt>diff, poseGt>dist, poseGt>dist2
%
% Cascaded Pose Regression Toolbox      Version 1.00
% Copyright 2012 Piotr Dollar.  [pdollar-at-caltech.edu]
% Please email me if you find bugs, or have suggestions or questions!
% Licensed under the Simplified BSD License [see bsd.txt]

%#ok<*DEFNU>
varargout = cell(1,max(1,nargout));
[varargout{:}] = feval(action,varargin{:});
end

function model = createModel( type )
% Create pose model (model is necessary for all other actions).
model=struct(); r=createPart(0); p=createPart(1);
switch type
  case {'ellipse','mouseTop','fish'} % single part model
    r.lks(:)=0; parts=r;
  case 'faces' % single part model w special display
    r.mus=[35 35 pi/2 5.2 -.1]; r.lks(:)=0; r.sigs=[5 5 .15 .3 .15];
    parts=r; model.isFace=1;
  case 'fishPose' % three part model for fish
    r.mus=[84 84 0 5.6 -2]; p.mus(2:3)=[0 -2]; p1=p; p2=p;
    p1.joint=[-1 0]; p2.joint=[1 0]; p2.mus(1)=0; parts=[r p1 p2];
    model.isFishPose=1;
  case 'mouseTopPose' % two part model for top view mouse
    r.lks(:)=0; r.mus=[112 112 0 6.45 -.45]; r.sigs=[15 15 pi .4 .4];
    p.mus=[0 -.6 -.5]; p.sigs=[pi/4 .36 0]; p.joint=[.6 0]; p.lks=[0 0 1];
    parts=[r p];
  case 'toyLizard' % articulated toy lizard 8 part model
    r.mus=[125 125 0 6  -1.5]; r.sigs=[10 10 pi .5 0]; pT1=p; pT2=p;
    pT1.mus=[pi -1.1 -1.4]; pT1.sigs(1)=pi*.6; pT1.joint=[-.8 0];
    pT2.mus=[0 0 -1.4]; pT2.sigs(1)=pi*.6; pT2.joint=[.5 0]; pT2.prn=2;
    pHd=p; pHd.mus=[0 -1.2 -1]; pHd.sigs=[pi/4 0 0]; pHd.joint=[.7 0];
    pLeg=p; pLeg.mus=[0 -1.6 -1]; pLeg.sigs=[pi/3 0 0];
    pFr=pLeg; pFr.mus(1)=-pi*.3; pFr.joint=[ .4  .5];
    pFl=pLeg; pFl.mus(1)= pi*.3; pFl.joint=[ .4 -.5];
    pBl=pLeg; pBl.mus(1)= pi*.6; pBl.joint=[-.35 -.5];
    pBr=pLeg; pBr.mus(1)=-pi*.6; pBr.joint=[-.35  .5];
    parts=[r pT1 pT2 pHd pFr pFl pBl pBr];
  otherwise
    error('unknown type: %s',type);
end
model.parts=parts;
end

function part = createPart( parent, wts )
% Create single part for model (parent==0 implies root).
if(nargin<2), wts=[0.313 0.342 11.339 13.059 6.998]; end
if( parent==0 )
  part = struct('prn',parent,'lks',[0 0 0 0 1],'joint',[],...
    'mus',[100 100 0 6 -1],'sigs',[10 10 pi .5 .5],'wts',wts);
else
  part = struct('prn',parent,'lks',[0 1 1],'joint',[0 -1],...
    'mus',[pi -1 -1],'sigs',[pi/2 .75 .5],'wts',wts(3:5));
end
end

function model = objSave( model, phis, fName )
% Save object part annotations to text file.
fid=fopen(fName,'w'); if(fid==-1), error('unable to open %s',fName); end
fprintf(fid,'%% poseGt version=2 m=%i\n',length(model.parts));
for i=1:size(phis,1), fprintf(fid,'%f ',phis(i,:)); fprintf(fid,'\n'); end
fclose(fid);
end

function phis = objLoad( model, fName )
% Load object part annotations from text file.
if(~exist(fName,'file')), error([fName ' not found']); end
type=textread(fName,'%% %s version=',1);
assert(strcmp(type,'poseGt') || strcmp(type,'bbGt'));
% special case to transparently load bbs created w bbGt
if(strcmp(type,'bbGt')), assert(length(model.parts)==1);
  phis=objsToPose(bbGt('bbLoad',fName)); return; end
fid=fopen(fName,'r'); if(fid==-1), error('unable to open %s',fName); end
out=fscanf(fid,'%% poseGt version=%d m=%d\n'); v=out(1); m=out(2);
assert(length(model.parts)==m); R=2+3*m;
if(v~=2), fclose(fid); error('Unknown version %i.',v); end
phis=zeros(10000,R); n=0; p=fscanf(fid,'%f ',R)';
while(~isempty(p)), n=n+1; phis(n,:)=p; p=fscanf(fid,'%f ',R)'; end
fclose(fid); phis=phis(1:n,:);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function h = draw( model, Is, phis, varargin )
% Draw object with parameters phis on top of Is.
dfs={'n',25, 'clrs','gcbm', 'drawIs',1, 'lw',1, 'is',[]};
[n,cs,drawIs,lw,is]=getPrmDflt(varargin,dfs,1);
% create single image I merging Is
n1=0; n2=0; if(length(n)>1), n2=n(1); n1=n(2); n=n1*n2; end
n=min(n,size(phis,1)); if(n1==0), n1=ceil(sqrt(n)); n2=ceil(n/n1); end
if(isempty(is)), is=1:n; end; phis=phis(is,:);
if(ndims(Is)==2), I=Is; d=zeros(n,2); else
  Is=Is(:,:,is); H=size(Is,1); W=size(Is,2); I=zeros(n2*H,n1*W,'uint8');
  d=(ind2sub2([n1 n2],1:n)-1); d(:,2)=d(:,2)*H; d(:,1)=d(:,1)*W;
  if(drawIs), for i=1:n, I((1:H)+d(i,2),(1:W)+d(i,1))=Is(:,:,i); end; end
end
% display I
if(drawIs), clf; im(I); colorbar off; axis off; title(''); axis('ij'); end
bbs=getBbs(model,phis,1); m=size(bbs,2); h=zeros(n,m,3); hold on;
if(ischar(cs)), cs=cs(:); end; if(size(cs,1)<m), cs=repmat(cs,m,1); end
for i=1:n, bbs(i,:,1)=bbs(i,:,1)+d(i,2); bbs(i,:,2)=bbs(i,:,2)+d(i,1); end
% special display for face model (draw face points)
if( isfield(model,'isFace') && model.isFace ), assert(m==1);
  yc=bbs(:,1); xc=bbs(:,2); yr=bbs(:,3); xr=bbs(:,4); ang=bbs(:,5);
  x1=xc-xr/2; x2=xc+xr/2; yd=(tan(ang+pi/2)).*(x2-x1)/2;
  y1=yc-yd; y2=yc+yd; y3=yr+yc; b=(2*y3-y1-y2)./(x2-x1);
  x3=(x1+x2)/2-b.*(y2-y1)/2; xs=[x1 x2 x3]; ys=[y1 y2 y3];
  xs=xs(:,[1:3 1]); ys=ys(:,[1:3 1]); h=zeros(n,1);
  for i=1:n, h(i)=plot(xs(i,:),ys(i,:),'.-','MarkerSize',20); end
  hold off; set(h,'LineWidth',lw,'Color',cs(1,:)); return
end
% special display for fishPose model (draw line segments and head)
if( isfield(model,'isFishPose') && model.isFishPose );
  cxt=bbs(:,2,2); cyt=bbs(:,2,1); cxh=bbs(:,3,2); cyh=bbs(:,3,1);
  ha=bbs(:,3,5); hl=bbs(:,3,3); ta=bbs(:,2,5); tl=bbs(:,2,3);
  x=zeros(n,4); y=zeros(n,4); h=zeros(n+3,1);
  x(:,1)=cxh+cos(ha).*hl; y(:,1)=cyh+sin(ha).*hl;
  x(:,2)=cxh+cos(ha+pi).*hl; y(:,2)=cyh+sin(ha+pi).*hl;
  x(:,3)=cxt+cos(ta+pi).*tl; y(:,3)=cyt+sin(ta+pi).*tl;
  x(:,4)=cxt+cos(ta).*tl; y(:,4)=cyt+sin(ta).*tl;
  for i=1:n, h(i)=plot(x(i,:),y(i,:),'-'); end
  h(n+1)=plot(x(:,1),y(:,1),'o','MarkerSize',5);
  h(n+2:n+3)=plot(x(:,2:3),y(:,2:3),'.','MarkerSize',10);
  hold off; h=h(:); set(h,'LineWidth',lw,'Color',cs(1,:)); return
end
% plot ellipses
for i=1:n, for j=1:m, b=bbs(i,j,:); [h(i,j,1),h(i,j,2),h(i,j,3)] = ...
      plotEllipse(b(1),b(2),b(3),b(4),b(5),cs(j,:)); end; end
set(h,'LineWidth',lw); hold off; h=h(:);
end

function [h,Vs] = drawRes( model, Is, phisGt, phis, varargin )
% Draw results overlaid on ground truth, separated into quantiles.
dfs={ 'quantile',5, 'nCol',6, 'lw',1, 'clrTr',0, 'ignGt',0, 'drawIs',1 };
[qnt,nCol,lw,clrTr,ignGt,drawIs]=getPrmDflt(varargin,dfs,1);
if(isempty(phisGt)), phisGt=phis(:,:,1); end
n=size(phis,1); T=size(phis,3); assert(size(phisGt,1)==n);
% sort by quality of results
ds=dist(model,phis,phisGt); [ds,ord]=sort(ds(:,1,end));
phisGt=phisGt(ord,:); phis=phis(ord,:,:);
% divide into quantiles
if(ndims(Is)==2), nShow=n; is=1:n; else Is=Is(:,:,ord);
  if(length(qnt)==1), qnt=round(linspace(0,n,qnt+1)); end
  q=length(qnt)-1; nCol=min(nCol,n); is=zeros(nCol,q); nShow=[q nCol];
  for i=1:q, is(:,i)=sort(randSample(qnt(i)+1:qnt(i+1),nCol,1)); end
end
% draw ground truth and results
if(clrTr), clrs=hsv(round(T*6)); clrs=clrs(round(T*.8):end,:);
else clrs=repmat([0 1 0],[T 1]); end;
h1=draw(model,Is,phisGt,'clrs',[0 .8 .8],'n',nShow,'is',is,'lw',lw,...
  'drawIs',drawIs); if(ignGt), delete(h1); h1=[]; end
for t=1:T
  h2=draw(model,Is,phis(:,:,t),'clrs',clrs(t,:),'drawIs',0,...
    'n',nShow,'is',is,'lw',lw); if(t==T), drawnow; break; end
  title(t); drawnow; if(nargout<2), delete(h2); continue; end
  if(t==1), pos=get(gca,'position'); else set(gca,'position',pos); end
  V=getframe(gca); V=V.cdata; if(t==1), Vs=zeros([size(V) T],'uint8'); end
  Vs(:,:,:,t)=V; delete(h2);
end; h=[h1(:) h2(:)];
if(nargout==2), V=getframe(gca); Vs(:,:,:,T)=V.cdata; end
end

function [Is,phis] = toyData( model, n, h, w, varargin )
% Generate toy images for testing each pose model.
dfs={'smooth',2,'noise',.2}; [smooth,noise]=getPrmDflt(varargin,dfs,1);
Is=zeros(h,w,n,'uint8'); f=filterGauss(4*smooth+1,[],smooth*smooth);
phis=random(model,n); I=zeros(h,w); masks=getMask(model,phis,h,w)>0;
for i=1:n
  M=masks(:,:,i); I(M)=.2+rand/5; I(~M)=.6+rand/5; I=I+randn(h,w)*noise;
  Is(:,:,i) = uint8(conv2(conv2(I,f,'same'),f','same')*255);
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function bbs = getBbs( model, phis, ellipse )
% Return [n x m x 5] absolute position for each object and each part.
if(nargin<3), ellipse=0; end
[xs,ys,ang,scl,asp]=getPose(model,phis); h=2.^scl; w=(2.^asp).*h;
if( ellipse ), bbs=cat(3,ys,xs,h/2,w/2,-ang); else
  bbs=cat(3,xs-w/2,ys-h/2,w,h,(pi/2-ang)/pi*180); end
end

function phi = setBbs( model, phi, id, bb )
% Set location of part id using given absolute position.
[xs,ys,ang,scl,asp]=getPose(model,phi); assert(size(phi,1)==1);
ang=ang(id)-(90-bb(5))/180*pi;
scl=scl(id)-log2(bb(4)); asp=asp(id)-log2(bb(3)/bb(4));
x=xs(id)-(bb(1)+bb(3)/2); y=ys(id)-(bb(2)+bb(4)/2);
% drag by changing x,y for root
if(id==1 || (abs(ang)<1e-10 && abs(scl)<1e-10 && abs(asp)<1e-10))
  if(~model.parts(1).lks(1)), phi(1)=phi(1)-x; end
  if(~model.parts(1).lks(2)), phi(2)=phi(2)-y; end
end
% alter scale of root if resizing part whose scale is locked
if( abs(scl)>1e-10 && model.parts(id).lks(2)==1 )
  phi(:,4)=phi(:,4)-scl; scl=0; end
% alter part as flags allow
p=model.parts(id); k=3*id; lks=p.lks(end-2:end);
d=[ang scl asp]; d(lks==1)=0; phi(k:k+2)=phi(k:k+2)-d;
end

function [xs,ys,ang,scl,asp,HS] = getPose( model, phis )
% Return cumulative pose for each part.
m=length(model.parts); n=size(phis,1); HS=zeros(2,3,n,m);
Z=zeros(n,m); xs=Z; ys=Z; ang=Z; scl=Z; asp=Z; init=zeros(1,m); k=0;
for p=1:m, part=model.parts(p);
  if(p==1), pc=phis(:,1:2); k=k+5;
    ang(:,p)=phis(:,3); scl(:,p)=phis(:,4); asp(:,p)=phis(:,5);
  else
    j=part.prn; assert(init(j)==1); ang(:,p)=phis(:,k+1)+ang(:,j);
    scl(:,p)=phis(:,k+2)+scl(:,j); asp(:,p)=phis(:,k+3); k=k+3;
  end; c=cos(-ang(:,p)); s=sin(-ang(:,p)); init(p)=1;
  t=2.^scl(:,p); s=s.*t; c=c.*t; t=2.^asp(:,p);
  HS(1,1,:,p)=c; HS(1,2,:,p)=t.*-s; HS(2,1,:,p)=s; HS(2,2,:,p)=t.*c;
  if(p>1), pc = multiTimes( HS(:,:,:,p), [1/2 0 1]', 1.1 ) + ...
      multiTimes( HS(:,:,:,part.prn), [part.joint/2 1]', 1.1 ); end
  xs(:,p)=pc(:,1); HS(1,3,:,p)=xs(:,p);
  ys(:,p)=pc(:,2); HS(2,3,:,p)=ys(:,p);
end
end

function masks = getMask( model, phis, h, w )
% Compute binary mask of pixels belonging to each object.
n=size(phis,1); m=length(model.parts); bbs=getBbs(model,phis,1);
masks=zeros(h,w,n); mm=@(e) maskEllipse(h,w,e(1),e(2),e(3),e(4),e(5));
for i=1:n, for j=1:m, masks(:,:,i)=masks(:,:,i)+mm(bbs(i,j,:)); end; end
masks(masks>0)=1;
end

function phis = objsToPose( objs )
% Convert objs created by bbGt to phis.
if(isempty(objs)), phis=zeros(0,5); return; end
bb=reshape([objs.bb],4,[])'; n=length(objs); phis=zeros(n,5);
phis(:,1:2)=bb(:,1:2)+bb(:,3:4)/2; phis(:,3)=-[objs.ang]/180*pi+pi/2;
phis(:,4:5)=log2([bb(:,4) bb(:,3)./bb(:,4)]); return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ftrData = ftrsGen( model, varargin )
% Generate random pose indexed features.
%
% USAGE
%  ftrData = ftrsGen( model, varargin )
%
% INPUTS
%  model    - pose model (see createModel())
%  varargin - additional params (struct or name/value pairs)
%   .type     - [2] feature type (1 or 2)
%   .F        - [100] number of ftrs to generate
%   .radius   - [2] sample initial x from circle of given radius
%   .nChn     - [1] number of image channels (e.g. 3 for color images)
%   .pids     - [] part ids for each x
%
% OUTPUTS
%  ftrData  - struct containing generated ftrs
%   .type     - feature type (1 or 2)
%   .F        - total number of features
%   .nChn     - number of image channels
%   .xs       - feature locations relative to unit circle
%   .pids     - part ids for each x
%
% EXAMPLE
%
% See also poseGt, poseGt>ftrsComp
dfs={'type',2,'F',100,'radius',2,'nChn',1,'pids',[]};
[type,F,radius,nChn,pids]=getPrmDflt(varargin,dfs,1);
F1=F*type; F2=max(100,ceil(F1*1.5)); xs=[]; m=length(model.parts);
while(size(xs,1)<F1), xs=rand(F2,2)*2-1; xs=xs(sum(xs.^2,2)<=1,:); end
xs=xs(1:F1,:)*radius/2; if(nChn>1), xs(:,3)=randint2(F1,1,[1 nChn]); end
if(isempty(pids)), pids=floor(linspace(0,F1,m+1)); end
ftrData=struct('type',type,'F',F,'nChn',nChn,'xs',xs,'pids',pids);
end

function [ftrData,fids1] = ftrsSubset( model, ftrData, fids0 ) %#ok<INUSL>
% Take subset of features according to fids.
assert(max(fids0(:))<=ftrData.F); %fids1=fids0; return
[kp,d,fids1]=unique(fids0(:)); F=length(fids0);
fids1=uint32(reshape(fids1,size(fids0)));
if(ftrData.type==2), kp=[kp*2-1 kp*2]'; kp=kp(:); end;
ftrData.xs=ftrData.xs(kp,:); ftrData.F=F; pids=ftrData.pids;
for p=1:length(pids)-1, s=pids(p)+1; e=pids(p+1);
  ftrData.pids(p+1)=ftrData.pids(p)+sum(kp>=s & kp<=e); end
end

function [ftrs,Vs] = ftrsComp( model, phis, Is, ftrData, imgIds )
% Compute pose indexed features on Is given phis.
%
% USAGE
%  [ftrs,Vs] = ftrsComp( model, phis, Is, ftrData, [imgIds] )
%
% INPUTS
%  model    - pose model
%  phis     - [MxR] relative pose for each image [tx ty sx sy theta]
%  Is       - [w x h x nCh x N] input images
%  ftrData  - define ftrs to actually compute, output of ftrsGen
%  imgIds   - [Mx1] image id for each phi (optional if M==N)
%
% OUTPUTS
%  ftrs     - [MxF] computed features
%  Vs       - [w x h x N] visualization of ftr locations
%
% EXAMPLE
%
% See also poseGt, poseGt>ftrsGen
w=size(Is,1); h=size(Is,2); nChn=ftrData.nChn;
if(nChn==1 && ndims(Is)<4), N=size(Is,3); else
  assert(size(Is,3)==nChn); N=size(Is,4); end
if(nargin<5 || isempty(imgIds)), imgIds=1:N; end
M=size(phis,1); assert(length(imgIds)==M);
m=length(model.parts); imgIds1=uint32(imgIds-1)*w*h*nChn;
% compute image inds from xs adjusted for pose
[d,d,d,d,d,HS] = getPose( model, phis );
inds=zeros(size(ftrData.xs,1),M,'uint32'); pids=ftrData.pids;
for p=1:m, s=pids(p)+1; e=pids(p+1); if(s>e), continue; end
  inds(s:e,:)=xsToInds(HS(:,:,:,p),ftrData.xs(s:e,:),w,h,nChn,imgIds1); end
% compute features
ftrs=double(Is(inds)')/255;
if(ftrData.type==1), ftrs=ftrs*2-1;
else ftrs=ftrs(:,1:2:end)-ftrs(:,2:2:end); end
% create V for visualization only
if(nargout==1), Vs=[]; return; end; Vs=double(Is); Vs=Vs/max(Vs(:));
if(nChn==1 && ndims(Vs)<4), Vs=permute(Vs,[1 2 4 3]); end; Vs0=Vs; Vs1=Vs;
Vs0(inds)=0; Vs1(inds)=1; Vs=cat(3,Vs0(:,:,1,:),Vs1(:,:,1,:),Vs0(:,:,1,:));
end

function inds = xsToInds( HS, xs, w, h, nChn, imgIds )
% Get inds into I given by x in coord system specified by H
if(isempty(xs)), inds=[]; return; end; m=size(HS,3);
O=ones(size(xs,1),1); xs=[xs O];
if(nChn>1), chs=xs(:,3); xs=xs(:,1:2); end
xs=reshape(permute(HS(1:2,:,:),[1 3 2]),m*2,3)*xs';
xs=permute(reshape(xs,2,m,[]),[3 2 1]);
cs=max(1,min(h,xs(:,:,1))); rs=max(1,min(w,xs(:,:,2)));
inds = uint32(rs) + uint32(cs-1)*w + imgIds(O,:);
if(nChn>1), inds = inds+uint32(chs-1)*w*h; end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function phis = identity( model, N )
% Return N copies of the identity element.
if(nargin<2), N=1; end; R=2+3*length(model.parts); phis=zeros(N,R);
end

function phis = random( model, N )
% Generate N random phis according to the model.
if(nargin<2), N=1; end; parts=model.parts; R=2+3*length(parts);
mus=[parts.mus]; sigs=[parts.sigs]; lks=[parts.lks]; sigs(lks==1)=0;
O=ones(N,1); phis = mus(O,:) + (rand(N,R)-.5) .* sigs(O,:);
phis(:,3:3:end) = normAng(phis(:,3:3:end),2*pi);
end

function phis = compose( model, phis0, phis1 ) %#ok<INUSL>
% Compose phis0 and phis1: phis = phis0 + phis1.
[N,R]=size(phis0); phis = phis0 + phis1;
if( 1 && any(any(phis0(:,1:4)~=0)) )
  H0=phisToHs(phis0(:,1:4)); H1=phisToHs(phis1(:,1:4));
  H=multiTimes(H0,H1,2); phis(:,1:4)=phisFrHs(H);
end
phis(:,3:3:R) = normAng(phis(:,3:3:R),2*pi);
end

function phis = inverse( model, phis0 ) %#ok<INUSL>
% Compute inverse of phis0 so that phis0+phis1=phis1+phis0=identity.
[N,R]=size(phis0); phis = -phis0;
if( 1 && any(any(phis0(:,1:4)~=0)) )
  H=phisToHs(phis0(:,1:4));
  Hi=multiDiv(H,eye(3),1); phis(:,1:4)=phisFrHs(Hi);
end
phis(:,3:3:R) = normAng(phis(:,3:3:R),2*pi);
end

function phi = compPhiStar( model, phis ) %#ok<INUSL>
% Compute phi that minimizes sum of distances to phis.
phi=mean(phis,1); [N,R]=size(phis); M=1000; as=linspace(0,2*pi,M);
if( 1 ) % numerical solution
  for r=3:3:R, del=normAng(phis(:,r*ones(1,M))-as(ones(1,N),:),2*pi);
    [v,ind]=min(sum(del.^2,1)); phi(r)=as(ind); end
else % closed form approximate solution
  is=3:3:R; s=mean(sin(phis(:,is)),1); c=mean(cos(phis(:,is)),1);
  phi(is)=mod(atan2(s,c),2*pi);
end
end

function del = diff( phis0, phis1 )
% Compute diffs between phis0(i,:,t) and phis1(i,:) for each i and t.
[N,R,T]=size(phis0); assert(size(phis1,3)==1);
del = phis0-phis1(:,:,ones(1,1,T));
del(:,3:3:R,:) = normAng(del(:,3:3:R,:),2*pi);
end

function [ds,dsAll] = dist( model, phis0, phis1 )
% Compute distance between phis0(i,:,t) and phis1(i,:) for each i and t.
[N,R,T]=size(phis0); wts=[model.parts.wts]; del=diff(phis0,phis1);
dsAll = del .* wts(ones(N,1),:,ones(T,1));
dsAll = dsAll.^2; ds=sum(dsAll,2)/R;
end

function D = dist2( model, phis0, phis1 )
% Compute distance between phis0(i,:) and phis1(j,:) for each i and j.
if( nargin<3 ), phis1=phis0; end; phis1=permute(phis1,[3 2 1]);
N=size(phis0,1); M=size(phis1,3); D=zeros(N,M);
for i=1:N, D(i,:)=dist(model,phis1,phis0(i,:)); end;
end

function Hs = phisToHs( phis4 )
% Compute rigid homography matrices from phis4 (trn,ang,scl).
[N,R]=size(phis4); assert(R==4); Hs=zeros(3,3,N); Hs(3,3,:)=1;
sc=2.^phis4(:,4); c=cos(phis4(:,3)).*sc; s=sin(phis4(:,3)).*sc;
Hs(1,3,:)=phis4(:,2); Hs(2,3,:)=phis4(:,1);
Hs(1,1,:)=c; Hs(1,2,:)=-s; Hs(2,1,:)=s; Hs(2,2,:)=c;
end

function phis4 = phisFrHs( Hs )
% Compute phis4 from rigid homography matrices (trn,ang,scl).
t=permute(Hs([2 1],3,:),[2 1 3]); a=atan2(Hs(2,1,:),Hs(1,1,:));
ss=Hs(1,1,:); sc=Hs(1,2,:); s=log2(sqrt(sc.*sc+ss.*ss));
phis4=permute([t a s],[3 2 1]);
end

function ang = normAng( ang, rng )
% Normalize angles so they fall in (-rng/2,rng/2]
ang=ang/rng+1000; ang=(ang-floor(ang))*rng;
ids=ang>rng/2; ang(ids)=ang(ids)-rng;
end
