require 'inline'

  class Robot

    inline(:C) do |builder|
      # builder.include '</home/rails/rails_project/app/classes/CGIKcore.cpp>'
      builder.add_compile_flags '-x c++', '-lstdc++'

      builder.c "
      int hello() {
        return 1;
      }
    "
    end

  end