function plot_nextpose(P_next, basicInfo)

board_Width = basicInfo.board_Width;
board_Height = basicInfo.board_Height;
image_Width = basicInfo.image_Width;
image_Height = basicInfo.image_Height;

figure; 
hold on;
for i = 1 : board_Height
    for j = 1 : board_Width
        pos = j + (i - 1) * board_Width;
        plot(P_next(1,pos), P_next(2,pos),'ro');
    end
end
plot(P_next(1,:), P_next(2,:),'b-');
xlim([0, image_Width]);
ylim([0, image_Height]);
set(gca,'Ydir','reverse')
drawnow
hold off;
end