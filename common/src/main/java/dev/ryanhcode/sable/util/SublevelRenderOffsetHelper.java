package dev.ryanhcode.sable.util;

import com.mojang.blaze3d.vertex.PoseStack;
import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.api.SubLevelHelper;
import dev.ryanhcode.sable.companion.math.Pose3dc;
import dev.ryanhcode.sable.sublevel.ClientSubLevel;
import dev.ryanhcode.sable.sublevel.SubLevel;
import net.minecraft.client.Minecraft;
import net.minecraft.world.phys.Vec3;
import org.joml.Quaterniondc;
import org.joml.Quaternionf;
import org.joml.Vector3dc;

public class SublevelRenderOffsetHelper {
    /**
     * @param subLevel Sub-level to move PoseStack from
     * @param ps       Pose stack to be manipulated
     * @usage Called before the PoseStack is translated to its plot position, or before anything is rendered with the plot position
     * @implNote The absolute camera position must never be multiplied into the (float) pose stack: at large
     * coordinates the float rounding of the camera translation quantizes to whole ULP steps and the rendered
     * geometry jitters whenever the camera moves. Everything here stays camera-relative in double precision;
     * call sites cancel the camera term via {@link #translation(Vec3)} instead.
     */
    public static void posePlotToProjected(final SubLevel subLevel, final PoseStack ps) {
        if (subLevel != null) {
            final Vec3 camera = Minecraft.getInstance().gameRenderer.getMainCamera().getPosition();

            final Pose3dc pose = ((ClientSubLevel) subLevel).renderPose();

            final Vector3dc pos = pose.position();
            final Vector3dc scale = pose.scale();
            final Quaterniondc orientation = pose.orientation();

            ps.translate(pos.x() - camera.x, pos.y() - camera.y, pos.z() - camera.z);
            ps.mulPose(new Quaternionf(orientation));
            ps.scale((float) scale.x(), (float) scale.y(), (float) scale.z());
        }
    }

    /**
     * @return rotation point of the given position's sub-level render pose, offset by the negative camera
     * position. Subtracting this from a camera-relative world coordinate (or adding it to the camera, or
     * moving an AABB by its negation) yields the plot-local coordinate that {@link #posePlotToProjected}
     * expects, without ever materializing the absolute camera position in float precision.
     */
    public static Vec3 translation(final Vec3 center) {
        final ClientSubLevel subLevel = Sable.HELPER.getContainingClient(center);
        if (subLevel != null) {
            final Vec3 camera = Minecraft.getInstance().gameRenderer.getMainCamera().getPosition();
            final Vector3dc rotationPoint = subLevel.renderPose().rotationPoint();
            return new Vec3(rotationPoint.x() - camera.x, rotationPoint.y() - camera.y, rotationPoint.z() - camera.z);
        }
        return Vec3.ZERO;
    }
}
