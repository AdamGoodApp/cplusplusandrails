require 'inline'

  class Robot

    inline(:C) do |builder|
      builder.include '</home/rails/cplusplus/app/classes/CGIKcore.cpp>'
      builder.add_compile_flags '-x c++', '-lstdc++'

      builder.c "
      double hello() {
        return MathTypes::Constants::pi;
      }
    "
    end

  end