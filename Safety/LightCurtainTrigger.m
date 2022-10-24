function [hit] = LightCurtainTrigger(objectmovement,object)
    
    curtainPole1 = [-1.6,0.845,-0.25];
    curtainPole2 = [1.8,0.845,-0.25];    
    curtainPole3 = [1.8,0.845,2.3]; % [X,Z,Y] ply coordinates

    curtain1 = [-1.6, -0.25, 0.845]; % [X,Y,Z]
    curtain2 = [1.8, 0.25, 0.845];    
    curtain3 = [1.8, 2.3, 0.845];

    switch object
        case 1 %For Hand going to the workspace
            hit = false;
            centerpoint = [0 0]; % Hand centerpoint
            handmovement = [objectmovement(1) - centerpoint(1), objectmovement(2), objectmovement(3)];
 
            %check the whole width of the boy
            for i = 0.0275:-0.0025:-0.0275
                %Check the whole length of the boy
                for j = 0.03:-0.0025:-0.03
                    %check if the objectmovement enters the workspace
                    if((handmovement (1,1)+i) > safetyBarrierPoint1(1,1) && (handmovement (1,1)+i) < safetyBarrierPoint4(1,1))
                        %display("1st");
                        if((handmovement (1,2)+j) > safetyBarrierPoint1(1,2) && (handmovement (1,2)+j) < safetyBarrierPoint4(1,2))
                            %display("2nd");
                            %Checks if object is past the light curtain
                            if((handmovement(1,1)+i) < curtain1(1,1) || (handmovement(1,1)+i) > curtain4(1,1))
                                %display("3rd");
                                if((handmovement(1,2)+j) > curtain1(1,2) || (handmovement(1,2)+j) > curtain2(1,2))
                                    %display("4th");
                                    hit = true;
                                end
                            end
                        end
                    end
                end
            end


        case 2 %Check if joint are outside the light curtain area
            hit = false;
%             display(objectmovement(1,4)+ " "+ objectmovement(2,4));
            if(objectmovement (1,4) < safetyBarrierPoint1(1,1) || objectmovement (1,4) > safetyBarrierPoint4(1,1))
                hit = true; %Out Of Bounds   
            end                
            if(objectmovement (2,4) < safetyBarrierPoint1(1,2) || objectmovement (2,4) > safetyBarrierPoint4(1,2))                    
                hit = true; %Out Of Bounds                
            end

end