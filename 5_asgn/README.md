
        Welcome to my CPE 500 final project: <b>Angry Birds Explosions!</b>
        
        <br>
        <br>
        Use these keys to control the simulation:
        <br>
        <br>
        <br>
        <b>1, 2, and 3:</b> Change scenes. <b>1</b> is a 3-layer tower, <b>2</b> is a tall, skinny tower and <b>3</b> is a little dude standing next to a wall.

        <br>
        <br>
        <div class="row">
            <div class="col-xs-4 col-xs-offset-2" />
                <img src="http://i.giphy.com/xT8qAZQjM5XDlm4RhK.gif" style="display: block"/>
            </div>
        </div>
        <br>
        <br>

        You can drag the mouse around to change where the "missiles" fly from. Once you press <b>space</b> to launch a missile, the mouse will start to control the camera instead!
        
        <div class="row">
            <div class="col-xs-4 col-xs-offset-2" />
                <img src="http://i.giphy.com/26BRrkJSoJTzbUSKQ.gif" style="display: block"/>
            </div>
        </div>

        Once a missile is flying, press <b>E</b> to cause an <b>Explosion!</b> All the block around any spawned missiles will get sent flying back!

			<div class="row">
            <div class="col-xs-4 col-xs-offset-2" />
                <img src="http://i.giphy.com/l0CRC2ld2Xnlncicw.gif" style="display: block"/>
            </div>
        </div>

        You can spawn as many missiles and cause as many explosions as you want. Go crazy! Once you change scenes, everything will be reset.

        <br>
        

        <h3>Explosion Forces</h3>
        There are a couple ways you could model explosions. You could simply set the velocity of nearby rigid bodies, but that might not be consistent if there are a lot of moving parts. I chose to apply an instant force to all objects caught in the explosion.
        <br>
        <pre>
function explode() {
   Vector3d target_posn = bodies[i].curr_E.block<3, 1>(0, 3);
   Vector3d bomb_posn = bodies[bomb_ndx].curr_E.block<3, 1>(0, 3);
   Vector3d diff = bomb_posn - target_posn;
   double dist = diff.norm();
   
   double radius = 10.0;
   if (dist > radius) dist = radius;
   
   Vector4d world_exp_force;
   world_exp_force << -diff.normalized() * (radius - dist) * 1000, 0.0;
   
   bodies[j].explosion_force = (bodies[j].curr_E.inverse() * world_exp_force).head(3);
}
</pre>
        <br>
        To do this, we take the vector FROM the bomb TO every rigid body in the scene. Then, we normalize it, and set its magnitude so that <i>close-by</i> bodies are affected more by the explosion.
        <br>
        <br>
        Then, we tell each rigid body an instantaneous force to apply to the next frame! Notice that since the force needs to be in the local space of the rigid body, we multiply the inverse of its E matrix to go from world space to local space. Neat!

      </div>
