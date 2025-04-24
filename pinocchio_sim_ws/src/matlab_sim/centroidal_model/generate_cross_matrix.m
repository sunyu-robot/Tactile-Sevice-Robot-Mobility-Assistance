function cross_matrix = generate_cross_matrix(v)
    % 输入：向量v，元素个数为3的向量
    % 输出：cross_matrix，叉乘矩阵

    % 检查输入向量是否有3个元素
    if length(v) ~= 3
        error('输入向量必须有3个元素');
    end

    cross_matrix = [0, -v(3), v(2);
                    v(3), 0, -v(1);
                    -v(2), v(1), 0];
end