function start_nodes = Multi_sample(Env, break_pts, num_node, IsDEBUG)
    for jdx = 1:length(break_pts) - 1
        start_nodes{jdx} = Env.sample_pts(break_pts(jdx), num_node);
        if IsDEBUG
            poses = cat(1, start_nodes{jdx}.pose);
            scatter(poses(:,1), poses(:,2))
            quiver(poses(:,1), poses(:,2), poses(:,3), poses(:,4))
        end
    end
end