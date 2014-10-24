function [ next_color ] = nextColor(curr_color)
% returns the next color in the repeated pattern
% green, brown, blue, yellow

% if input 0 is given, functions returns color of LH leg

green  = [0.0 0.85 0.0]; % left-hind
brown  = [0.7 0.5 0.0]; % left-front
blue   = [0.0 0.0 1.0]; % right-hind
yellow = [0.95 0.95 0]; % right-front

% LH, LF, RH, RF
% by color definition
if isequal(curr_color,green)
    next_color = brown;
elseif isequal(curr_color,brown)
    next_color = blue;
elseif isequal(curr_color,blue)
    next_color = yellow;
elseif isequal(curr_color,yellow)
    next_color = green;
    
% by color string
elseif strcmp(curr_color,'green')
    next_color = brown;
elseif strcmp(curr_color,'brown')
    next_color = blue;
elseif strcmp(curr_color,'blue')
    next_color = yellow;
elseif strcmp(curr_color,'yellow')
    next_color = green;
    
end

