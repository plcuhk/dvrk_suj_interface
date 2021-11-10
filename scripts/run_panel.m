suj1_sub = rossubscriber('/dvrk/PSM1/SUJ/joint_states'); 
suj2_sub = rossubscriber('/dvrk/PSM2/SUJ/joint_states'); 
suj3_sub = rossubscriber('/dvrk/ECM/SUJ/joint_states'); 



%suj1_pub = rospublisher('/dvrk/PSM1/SUJ/is_brake_release_list');
App = SUJ_panel;


suj1_release_list = [false,false,false,false,false,false];
suj2_release_list = [false,false,false,false,false,false];
suj3_release_list = [false,false,false,false];

% udate msg
while(true)
    %% SUJ1
    % get msg form subscriber
    msg = receive(suj1_sub);
    pos = msg.Position;
    App.SUJ1Joint1Gauge.Value = pos(1)*100;
    App.SUJ1Joint2Gauge.Value = pos(2);
    App.SUJ1Joint3Gauge.Value = pos(3);
    App.SUJ1Joint4Gauge.Value = pos(4);
    App.SUJ1Joint5Gauge.Value = pos(5);
    App.SUJ1Joint6Gauge.Value = pos(6);
    temp = [];
    temp(1) = App.SUJ1ReleaseButton_Joint1.Value ~=0;
    temp(2) = App.SUJ1ReleaseButton_Joint2.Value ~=0;
    temp(3) = App.SUJ1ReleaseButton_Joint3.Value ~=0;
    temp(4) = App.SUJ1ReleaseButton_Joint4.Value ~=0;
    temp(5) = App.SUJ1ReleaseButton_Joint5.Value ~=0;
    temp(6) = App.SUJ1ReleaseButton_Joint6.Value ~=0;
    if ~all(suj1_release_list ==  temp)
%         suj1_release_list = temp;
%         msg = rosmessage(suj1_pub);
%         msg.data = suj1_release_list;
%         send(suj1_pub, msg);
    end
    
    %% SUJ2
    % get msg form subscriber
    msg = receive(suj2_sub);
    pos = msg.Position;
    App.SUJ2Joint1Gauge.Value = pos(1)*100;
    App.SUJ2Joint2Gauge.Value = pos(2);
    App.SUJ2Joint3Gauge.Value = pos(3);
    App.SUJ2Joint4Gauge.Value = pos(4);
    App.SUJ2Joint5Gauge.Value = pos(5);
    App.SUJ2Joint6Gauge.Value = pos(6);
    temp = [];
    temp(1) = App.SUJ2ReleaseButton_Joint1.Value ~=0;
    temp(2) = App.SUJ2ReleaseButton_Joint2.Value ~=0;
    temp(3) = App.SUJ2ReleaseButton_Joint3.Value ~=0;
    temp(4) = App.SUJ2ReleaseButton_Joint4.Value ~=0;
    temp(5) = App.SUJ2ReleaseButton_Joint5.Value ~=0;
    temp(6) = App.SUJ2ReleaseButton_Joint6.Value ~=0;
    if ~all(suj2_release_list ==  temp)
%         suj1_release_list = temp;
%         msg = rosmessage(suj1_pub);
%         msg.data = suj1_release_list;
%         send(suj1_pub, msg);
    end
 
    %% ECM
    % get msg form subscriber
    msg = receive(suj3_sub);
    pos = msg.Position;
    App.SUJ3Joint1Gauge.Value = pos(1)*100;
    App.SUJ3Joint2Gauge.Value = pos(2);
    App.SUJ3Joint3Gauge.Value = pos(3);
    App.SUJ3Joint4Gauge.Value = pos(4);
    temp = [];
    temp(1) = App.SUJ3ReleaseButton_Joint1.Value ~=0;
    temp(2) = App.SUJ3ReleaseButton_Joint2.Value ~=0;
    temp(3) = App.SUJ3ReleaseButton_Joint3.Value ~=0;
    temp(4) = App.SUJ3ReleaseButton_Joint4.Value ~=0;
    if ~all(suj3_release_list ==  temp)
%         suj1_release_list = temp;
%         msg = rosmessage(suj1_pub);
%         msg.data = suj1_release_list;
%         send(suj1_pub, msg);
    end
end







