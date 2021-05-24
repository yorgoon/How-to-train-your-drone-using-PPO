classdef preluLayer < nnet.layer.Layer
    % Example custom PReLU layer.

    properties (Learnable)
        % Layer learnable parameters
            
        % Scaling coefficient
        Alpha
    end
    
    methods
        function layer = preluLayer(numChannels, name) 
            % layer = preluLayer(numChannels, name) creates a PReLU layer
            % for 2-D image input with numChannels channels and specifies 
            % the layer name.

            % Set layer name.
            layer.Name = name;

            % Set layer description.
            layer.Description = "PReLU with " + numChannels + " channels"; 
            % Initialize scaling coefficient.
            layer.Alpha = rand; 
        end
        
        function Z = predict(layer, X)
            % Z = predict(layer, X) forwards the input data X through the
            % layer and outputs the result Z.
            thrust = 1.477*9.807/4;
            fx1 = (tanh((X+1.5))+1)*(thrust-1)/2 + 1;
            fx2 = (tanh((X-1.5))+1)*(thrust-1)/2;
            Z = fx1+fx2;
%             Z = max(X,0) + layer.Alpha .* min(0,X);
        end
    end
end