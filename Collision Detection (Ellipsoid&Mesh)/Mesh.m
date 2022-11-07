classdef Mesh < handle
    properties (Access = public)
        position; % Set the position of the mesh
        meshLength;
        meshDensity;
        sideLength;
        meshPoints;
        plot_h;
    end
    methods 
        function self = Mesh(meshLength, meshDensity, position)
            self.updateParameters(meshLength, meshDensity, position)
        end
        
        function updateParameters(self, meshLength, meshDensity, position)
            self.meshLength = meshLength;
            self.meshDensity = meshDensity;
            self.position = position;
        end
        
        function removePlots(self)
            self.plot_h.reset();
            refreshdata
            drawnow            
        end
        
        function updatePlot(self)
            self.sideLength = -(self.meshLength/2):(self.meshLength/self.meshDensity):(self.meshLength/2);
            [Y,Z] = meshgrid(self.sideLength,self.sideLength);
            sizeMat = size(Y);
            X = repmat(self.meshLength/2,sizeMat(1),sizeMat(2));
            
            % Combine one surface as a point cloud
            self.meshPoints = [X(:),Y(:),Z(:)];
            self.meshPoints = [ self.meshPoints ...
                         ; self.meshPoints * rotz(pi/2)...
                         ; self.meshPoints * rotz(pi) ...
                         ; self.meshPoints * rotz(3*pi/2) ...
                         ; self.meshPoints * roty(pi/2) ...
                         ; self.meshPoints * roty(-pi/2)];         
            self.meshPoints = self.meshPoints + repmat(self.position,size(self.meshPoints,1),1);
            try
                self.plot_h.reset();
            end
            self.plot_h = plot3(self.meshPoints(:,1),self.meshPoints(:,2),self.meshPoints(:,3),'r.');
            refreshdata
            drawnow
        end
        
        function move(self,position)
            self.position = position;
            self.updatePlot()
        end
        
        function points = getPoints(self)
            points = self.meshPoints;
        end
    end
end