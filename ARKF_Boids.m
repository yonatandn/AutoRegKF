%% Load Traject
clear all
close all
clc
cd('C:\Users\yonat\Documents\Academic\Final_Project\05 Code\01 Matlab\AutoRegKF')
Save_Video = 0;

%% Ground Truth
[filename, pathname] = uigetfile()
Traj = load([pathname filename]); Traj = Traj.Traj;
SimTime = size(Traj.boids,1);

%% Auto-Regressive
n = size(Traj.boids,2); % Number of boids
k = 20; % itiration time history
d = size(Traj.boids,3); % Dimension of Problem
nd2k = k*(n*d)^2;
nd = n*d;
ndk = n*d*k;
I=eye(nd2k);

Start_Pixel = 150;

Q =10;      % TBD
R =0.1;     % TBD
P =1000;    % TBD

% Model
A  = eye(nd2k);
xk = zeros(nd2k,1);   % X0 [ax_t-1,ax_t-2,bx_t-1,bx_t-2,ay_t-1,ay_t-2,by_t-1,by_t-2]
% xk(1:d:end) = 1;       % X0 = [1 0 1 0 ..]
xk = 0.5*ones(nd2k,1); xk = xk/norm(xk,2);
Pk = P*ones(size(A)); clear P  % x's Covariance
Q = Q*eye(nd2k);
R = R*eye(nd);

%% Build Measurment
sigma_meas = 0;
Meas = Traj.boids+normrnd(0,sigma_meas) ;
MeasFreq = 1; % [Hz]
HistorySize = 0;

%% Initiallize Structures
History = zeros(1, ndk);
H = Start_Pixel * (2/nd2k) *ones(n*d,nd2k);
Sol.t=zeros(1,SimTime);
Sol.X=zeros(nd,SimTime);
Sol.P=zeros(nd2k,SimTime);
Sol.weights=zeros(nd2k,SimTime);
Sol.Meas.t=[];
Sol.Meas.z=[];

ii=1;
Sol.t(1,ii)=0;
Sol.X(:,ii)=H*xk;
Sol.P(:,ii)=diag(Pk);
Sol.weights(:,ii)=xk;

%% Other Params
Pred_Size = 20;
X_forecasted = zeros(nd,Pred_Size);
Colors = rand(n,3);

%% Video Settings
if Save_Video
    % Create video writer object.
    video = VideoWriter('Video.avi', 'Motion JPEG AVI');
    % Set the frame rate.
    video.FrameRate = 20;
    % Open the video file.
    open(video);
end

%% Solution
fig555 = createFig();

% dt = 0.1; % [sec]
dt=1; % [sec]
for t=dt:dt:SimTime
    ii=ii+1;

    xk1=A*xk;
    Pk1=A*Pk*A'+Q;

    % Check for new measurement
    if (mod(t,1/MeasFreq)==0)
        z = reshape(squeeze(Meas(t,:,:))',[n*d,1]); % Arrange into format [x1,y1,z1,x1,y2,z2...]
        HistorySize = HistorySize+1;
        if (HistorySize > k)
            xk=xk1;
            Pk=Pk1;
            HCell = repmat({History}, n, d); % Create H in the format [History 0 0 0 0; 0 0 0 0 History]
            H = blkdiag(HCell{:});
            clear HCell
            y=z-H*xk1;
            S=H*Pk*H'+R;
            K=Pk*H'/S;
            xk1=xk+K*y;
            Pk1=(I-K*H)*Pk;
        end
        History = [z' History];
        History = History(1:end-nd);
        Sol.Meas.t=[Sol.Meas.t t];
        Sol.Meas.z=[Sol.Meas.z z];
    end

    xk=xk1;
    Pk=Pk1;

    Sol.t(1,ii)=t;
    Sol.X(:,ii)=H*xk;
    Sol.P(:,ii)=diag(Pk);
    Sol.weights(:,ii)=xk;

    % Forecasting
    if (HistorySize > k)
        History4Pred = History;
        H_Pred = H;
        for time_ind = 1:Pred_Size
            X_forecasted(:,time_ind) = H_Pred*xk;
            History4Pred = [X_forecasted(:,time_ind)' History4Pred];
            History4Pred = History4Pred(1:end-nd);
            HCell = repmat({History4Pred}, n, d); % Create H in the format [History 0 0 0 0; 0 0 0 0 History]
            H_Pred = blkdiag(HCell{:});
            clear HCell
        end

        % Plots - 2D
        for jj = 1:n
            plot(Traj.boids(ii-k:ii-1,jj,1),Traj.boids(ii-k:ii-1,jj,2),'.k')%,'Color',Colors(ii,:),'MarkerSize',12)
            hold on
            plot(X_forecasted(1+2*(jj-1),:),X_forecasted(2+2*(jj-1),:),'.','Color',1-Colors(jj,:),'MarkerSize',12)
        end
        hold off
        xlim([0 300])
        ylim([0 300])
    end
    pause(0.01)

    if Save_Video
        frame = getframe(gcf);       % Set up the movie.
        writeVideo(video, frame);% Set up the movie.
    end

end

if Save_Video
    close(video);
end


%% Plot Figures
% 2-D Currently
figure(21)
for ii = 1:n
    temp = plot(Sol.X(1+2*(ii-1),:),Sol.X(2+2*(ii-1),:));
    hold on
    plot(Traj.boids(:,ii,1),Traj.boids(:,ii,2),'Color',temp.Color)
    grid on
    box on
    %     plot(Sol.Meas.z(1,:),Sol.Meas.z(2,:),'k.');
    title('AR KF Solution')
end

figure(23)
plot(Sol.t,Sol.weights(1,:))
title('Weights')
hold on
grid on
box on
for ii=2:size(Sol.weights,1)-1
    plot(Sol.t,Sol.weights(ii,:))
end

%%
function newfig = createFig()
    newfig = figure(555);
    hold on
    xlim([0 300])
    ylim([0 300])
    grid on
    box on
end