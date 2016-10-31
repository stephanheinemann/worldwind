/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package gov.nasa.worldwind.geom;


import com.cfar.swim.worldwind.geom.precision.PrecisionVec4;

import gov.nasa.worldwind.util.Logging;

/**
 * Realizes a transformation matrix with additional static methods for
 * coordinate transformations.
 * 
 * @author Stephan Heinemann
 *
 */
public final class TransformationMatrix extends Matrix {
	
	/**
	 * Constructs a new transform matrix from orthonormal rotation vectors.
	 * 
	 * @throws IllegalArgumentException if the rotation vectors are not
	 *         orthonormal
	 */
	public TransformationMatrix(
		double m11, double m12, double m13, double m14,
		double m21, double m22, double m23, double m24,
		double m31, double m32, double m33, double m34,
		double m41, double m42, double m43, double m44) {
		super(
			m11, m12, m13, m14,
			m21, m22, m23, m24,
			m31, m32, m33, m34,
			m41, m42, m43, m44,
			true);
		
		Vec4 s = new Vec4(m11, m21, m31);
		Vec4 u = new Vec4(m12, m22, m32);
		Vec4 f = new Vec4(m13, m23, m33);
		
		if (!(new PrecisionVec4(s, 2).areOrthonormal(u, f))) {
			// TODO: review message resource bundle
			String msg = Logging.getMessage("generic.NonOrthonormalVectors");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}
	}
	
	/**
	 * Constructs a transformation matrix from three orthonormal axes.
	 * 
	 * @param axes the three orthonormal axes
	 * 
	 * @return the transformation matrix from three orthonormal axes
	 * 
	 * @throws IllegalArgumentException if the axes are not orthonormal
	 */
	public static Matrix fromAxes(Vec4[] axes) {
		if (axes == null)
        {
            String msg = Logging.getMessage("nullValue.AxesIsNull");
            Logging.logger().severe(msg);
            throw new IllegalArgumentException(msg);
        }

        if (axes.length < 3)
        {
            String msg = Logging.getMessage("generic.ArrayInvalidLength", axes.length);
            Logging.logger().severe(msg);
            throw new IllegalArgumentException(msg);
        }

        if (axes[0] == null || axes[1] == null || axes[2] == null)
        {
            String msg = Logging.getMessage("nullValue.AxesIsNull");
            Logging.logger().severe(msg);
            throw new IllegalArgumentException(msg);
        }
        
        if (!(new PrecisionVec4(axes[0],2).areOrthonormal(axes[1], axes[2]))) {
        	String msg = Logging.getMessage("generic.NonOrthonormalVectors");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
        }

        Vec4 s = axes[0].normalize3();
        Vec4 u = axes[1].normalize3();
        Vec4 f = axes[2].normalize3();
        
        return new TransformationMatrix(
            s.x, u.x, f.x, 0.0,
            s.y, u.y, f.y, 0.0,
            s.z, u.z, f.z, 0.0,
            0.0, 0.0, 0.0, 1.0);
	}
	
	/**
	 * Constructs a transformation matrix that transform from a local to a
	 * global reference system with the specified origin and orthonormal axes
	 * of the local system.
	 * 
	 * @param origin the origin of the local reference system
	 * @param axes the orthonormal axes of the local reference system
	 * 
	 * @return the transformation matrix that transforms from the local to the
	 *         global reference system
	 */
	public static Matrix fromLocalOrientation(Vec4 origin, Vec4[] axes) {
		return Matrix.fromTranslation(origin).multiply(TransformationMatrix.fromAxes(axes));
	}
	
	/**
	 * Constructs a transformation matrix that transform from a global to a
	 * local reference system with the specified origin and orthonormal axes
	 * of the local system.
	 * 
	 * @param origin the origin of the local reference system
	 * @param axes the orthonormal axes of the local reference system
	 * 
	 * @return the transformation matrix that transforms from the global to the
	 *         local reference system
	 */
	public static Matrix toLocalOrientation(Vec4 origin, Vec4[] axes) {
		return TransformationMatrix.fromLocalOrientation(origin, axes).getInverse();
	}

}
