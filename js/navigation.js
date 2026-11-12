/* =========================================================================
   NAVIGATION GUIDE — Dynamic Pathfinding & Terrain Trajectory Ribbon
   Calculates an obstacle-avoiding route from player/start to any target,
   projects it seamlessly onto Martian terrain relief, and exposes
   query functions to measure the player's distance to the path.
   ========================================================================= */

(function(){
  'use strict';

  // State
  let pathPoints = [];           // Array of THREE.Vector3 conforming to terrain
  let rawWaypoints = [];         // Key拐 points from pathfinder
  let routeMesh = null;          // Three.js Ribbon Mesh
  let trailParticles = null;     // Floating directional pulse indicators
  let trailTexture = null;
  let activeTarget = null;       // { x, z }
  let activeStart = null;        // { x, z }
  let lastCalcTime = 0;
  let animationTime = 0;

  /* -------------------- Procedural Flow Texture -------------------- */
  function createTrailTexture(){
    if (trailTexture) return trailTexture;
    const canvas = document.createElement('canvas');
    canvas.width = 512;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');

    // Background gradient with glowing center beam and lateral falloff
    const grad = ctx.createLinearGradient(0, 0, 0, 64);
    grad.addColorStop(0, 'rgba(0, 220, 255, 0)');
    grad.addColorStop(0.2, 'rgba(0, 220, 255, 0.35)');
    grad.addColorStop(0.5, 'rgba(255, 255, 255, 0.95)');
    grad.addColorStop(0.8, 'rgba(0, 220, 255, 0.35)');
    grad.addColorStop(1, 'rgba(0, 220, 255, 0)');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, 512, 64);

    // Flow chevrons (> > >) pointing in direction of movement
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.85)';
    ctx.lineWidth = 4;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    for (let x = 0; x < 512; x += 64){
      ctx.beginPath();
      ctx.moveTo(x + 12, 14);
      ctx.lineTo(x + 36, 32);
      ctx.lineTo(x + 12, 50);
      ctx.stroke();

      // Soft glow dot in center of each chevron
      ctx.fillStyle = 'rgba(79, 209, 255, 0.9)';
      ctx.beginPath();
      ctx.arc(x + 36, 32, 5, 0, Math.PI * 2);
      ctx.fill();
    }

    trailTexture = new THREE.CanvasTexture(canvas);
    trailTexture.wrapS = THREE.RepeatWrapping;
    trailTexture.wrapT = THREE.ClampToEdgeWrapping;
    return trailTexture;
  }

  /* -------------------- Obstacle Collision Helpers -------------------- */
  function lineIntersectsObstacle(p1, p2, obstacle, margin = 1.0){
    const r = obstacle.radius + margin;
    const dx = p2.x - p1.x;
    const dz = p2.z - p1.z;
    const lenSq = dx * dx + dz * dz;
    if (lenSq === 0) return Math.hypot(p1.x - obstacle.x, p1.z - obstacle.z) < r;

    // Projection of obstacle onto segment
    const t = Math.max(0, Math.min(1, ((obstacle.x - p1.x) * dx + (obstacle.z - p1.z) * dz) / lenSq));
    const projX = p1.x + t * dx;
    const projZ = p1.z + t * dz;
    const distSq = (obstacle.x - projX) * (obstacle.x - projX) + (obstacle.z - projZ) * (obstacle.z - projZ);
    return distSq < r * r;
  }

  function isSegmentClear(p1, p2, obstacles, margin = 0.85){
    if (!obstacles || obstacles.length === 0) return true;
    for (let i = 0; i < obstacles.length; i++){
      if (lineIntersectsObstacle(p1, p2, obstacles[i], margin)) return false;
    }
    return true;
  }

  /* -------------------- 2D A* Pathfinding around rocks -------------------- */
  function findObstacleFreePath(start, target, obstacles){
    // 1. Direct path check (Fast path: if unobstructed, straight line suffices)
    if (isSegmentClear(start, target, obstacles, 0.9)){
      return [new THREE.Vector2(start.x, start.z), new THREE.Vector2(target.x, target.z)];
    }

    // 2. Discretized local grid A* around bounding box of start + target
    const minX = Math.min(start.x, target.x) - 16;
    const maxX = Math.max(start.x, target.x) + 16;
    const minZ = Math.min(start.z, target.z) - 16;
    const maxZ = Math.max(start.z, target.z) + 16;

    const cellSize = 1.2; // 1.2m cell grid resolution
    const cols = Math.ceil((maxX - minX) / cellSize);
    const rows = Math.ceil((maxZ - minZ) / cellSize);

    const startCol = Math.max(0, Math.min(cols - 1, Math.round((start.x - minX) / cellSize)));
    const startRow = Math.max(0, Math.min(rows - 1, Math.round((start.z - minZ) / cellSize)));
    const targetCol = Math.max(0, Math.min(cols - 1, Math.round((target.x - minX) / cellSize)));
    const targetRow = Math.max(0, Math.min(rows - 1, Math.round((target.z - minZ) / cellSize)));

    const key = (c, r) => `${c},${r}`;
    const blocked = new Set();

    // Mark cells overlapping with any obstacle
    for (let i = 0; i < (obstacles || []).length; i++){
      const obs = obstacles[i];
      const radius = obs.radius + 0.85;
      const cMin = Math.max(0, Math.floor((obs.x - radius - minX) / cellSize));
      const cMax = Math.min(cols - 1, Math.ceil((obs.x + radius - minX) / cellSize));
      const rMin = Math.max(0, Math.floor((obs.z - radius - minZ) / cellSize));
      const rMax = Math.min(rows - 1, Math.ceil((obs.z + radius - minZ) / cellSize));

      for (let c = cMin; c <= cMax; c++){
        for (let r = rMin; r <= rMax; r++){
          const wx = minX + c * cellSize;
          const wz = minZ + r * cellSize;
          if (Math.hypot(wx - obs.x, wz - obs.z) < radius){
            blocked.add(key(c, r));
          }
        }
      }
    }

    // Ensure start and target are not blocked
    blocked.delete(key(startCol, startRow));
    blocked.delete(key(targetCol, targetRow));

    // A* search
    const openSet = [{ col: startCol, row: startRow, g: 0, f: Math.hypot(startCol - targetCol, startRow - targetRow) }];
    const cameFrom = new Map();
    const gScore = new Map();
    gScore.set(key(startCol, startRow), 0);

    const neighbors = [
      { dc: 1, dr: 0, cost: 1.0 },
      { dc: -1, dr: 0, cost: 1.0 },
      { dc: 0, dr: 1, cost: 1.0 },
      { dc: 0, dr: -1, cost: 1.0 },
      { dc: 1, dr: 1, cost: 1.414 },
      { dc: -1, dr: 1, cost: 1.414 },
      { dc: 1, dr: -1, cost: 1.414 },
      { dc: -1, dr: -1, cost: 1.414 }
    ];

    let foundTarget = false;
    let iterations = 0;
    const maxIterations = 3500;

    while (openSet.length > 0 && iterations++ < maxIterations){
      openSet.sort((a, b) => a.f - b.f);
      const current = openSet.shift();
      const currentKey = key(current.col, current.row);

      if (current.col === targetCol && current.row === targetRow){
        foundTarget = true;
        break;
      }

      for (let i = 0; i < neighbors.length; i++){
        const n = neighbors[i];
        const nc = current.col + n.dc;
        const nr = current.row + n.dr;
        if (nc < 0 || nc >= cols || nr < 0 || nr >= rows) continue;
        const nKey = key(nc, nr);
        if (blocked.has(nKey)) continue;

        const tentG = (gScore.get(currentKey) || 0) + n.cost;
        if (tentG < (gScore.get(nKey) || Infinity)){
          cameFrom.set(nKey, { col: current.col, row: current.row });
          gScore.set(nKey, tentG);
          const h = Math.hypot(nc - targetCol, nr - targetRow);
          const f = tentG + h * 1.1;

          const existing = openSet.find(o => o.col === nc && o.row === nr);
          if (existing){
            existing.g = tentG;
            existing.f = f;
          } else {
            openSet.push({ col: nc, row: nr, g: tentG, f });
          }
        }
      }
    }

    // Reconstruct raw node path
    const rawNodes = [];
    if (foundTarget){
      let curr = { col: targetCol, row: targetRow };
      while (curr){
        rawNodes.unshift(new THREE.Vector2(minX + curr.col * cellSize, minZ + curr.row * cellSize));
        curr = cameFrom.get(key(curr.col, curr.row));
      }
      rawNodes[0] = new THREE.Vector2(start.x, start.z);
      rawNodes[rawNodes.length - 1] = new THREE.Vector2(target.x, target.z);
    } else {
      // Fallback: direct line with midpoint detours if path wasn't found within iteration limits
      return [new THREE.Vector2(start.x, start.z), new THREE.Vector2(target.x, target.z)];
    }

    // 3. String-pulling / Raycast Shortcut smoothing: skip unnecessary intermediate steps
    const smoothed = [rawNodes[0]];
    let anchorIdx = 0;
    while (anchorIdx < rawNodes.length - 1){
      let furthest = anchorIdx + 1;
      for (let j = rawNodes.length - 1; j > anchorIdx + 1; j--){
        if (isSegmentClear(rawNodes[anchorIdx], rawNodes[j], obstacles, 0.75)){
          furthest = j;
          break;
        }
      }
      smoothed.push(rawNodes[furthest]);
      anchorIdx = furthest;
    }

    return smoothed;
  }

  /* -------------------- Ribbon Geometry Generation -------------------- */
  function buildTerrainRibbonGeometry(points3D, width = 0.55){
    const numPoints = points3D.length;
    if (numPoints < 2) return new THREE.BufferGeometry();

    const positions = new Float32Array(numPoints * 2 * 3);
    const uvs = new Float32Array(numPoints * 2 * 2);
    const indices = [];

    let totalDist = 0;
    const dists = [0];
    for (let i = 1; i < numPoints; i++){
      totalDist += points3D[i].distanceTo(points3D[i - 1]);
      dists.push(totalDist);
    }

    const up = new THREE.Vector3(0, 1, 0);

    for (let i = 0; i < numPoints; i++){
      const curr = points3D[i];
      let dir;
      if (i === 0){
        dir = points3D[1].clone().sub(curr).normalize();
      } else if (i === numPoints - 1){
        dir = curr.clone().sub(points3D[i - 1]).normalize();
      } else {
        dir = points3D[i + 1].clone().sub(points3D[i - 1]).normalize();
      }

      // Normal to direction on horizontal plane
      const side = new THREE.Vector3(-dir.z, 0, dir.x).normalize().multiplyScalar(width * 0.5);

      const pLeft = curr.clone().add(side);
      const pRight = curr.clone().sub(side);

      const idx = i * 2;
      positions[idx * 3]     = pLeft.x;
      positions[idx * 3 + 1] = pLeft.y;
      positions[idx * 3 + 2] = pLeft.z;

      positions[(idx + 1) * 3]     = pRight.x;
      positions[(idx + 1) * 3 + 1] = pRight.y;
      positions[(idx + 1) * 3 + 2] = pRight.z;

      // UVs: V runs 0 to 1 across width; U stretches along path length
      const u = dists[i] * 0.7;
      uvs[idx * 2]     = u;
      uvs[idx * 2 + 1] = 0.0;

      uvs[(idx + 1) * 2]     = u;
      uvs[(idx + 1) * 2 + 1] = 1.0;

      if (i < numPoints - 1){
        const a = idx;
        const b = idx + 1;
        const c = idx + 2;
        const d = idx + 3;
        indices.push(a, b, c);
        indices.push(b, d, c);
      }
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    return geo;
  }

  /* -------------------- Main Navigation Module API -------------------- */
  const NavigationGuide = {
    enabled: true,

    /**
     * Compute and render a path from `start` to `target`.
     * Automatically projects onto terrain relief and avoids obstacles in `surfaceRocksData`.
     */
    calculatePath: function(start, target, opts = {}){
      if (!window.surfaceScene) return [];

      const startPt = start || (typeof surfacePos !== 'undefined' ? { x: surfacePos.x, z: surfacePos.z } : { x: 0, z: 0 });
      const targetPt = target || (window.storyTargetMesh ? { x: storyTargetMesh.position.x, z: storyTargetMesh.position.z } : null);

      if (!targetPt) return [];

      activeStart = { x: startPt.x, z: startPt.z };
      activeTarget = { x: targetPt.x, z: targetPt.z };

      const obstacles = opts.obstacles || window.surfaceRocksData || [];
      const seed = opts.seed || window.currentSurfaceSeed || 1;

      // 1. Calculate optimal 2D node sequence avoiding rocks
      rawWaypoints = findObstacleFreePath(activeStart, activeTarget, obstacles);

      // 2. Sample dense 3D points following terrain elevation
      pathPoints = [];
      if (rawWaypoints.length < 2){
        pathPoints = [
          new THREE.Vector3(activeStart.x, terrainHeight(activeStart.x, activeStart.z, seed) + 0.18, activeStart.z),
          new THREE.Vector3(activeTarget.x, terrainHeight(activeTarget.x, activeTarget.z, seed) + 0.18, activeTarget.z)
        ];
      } else {
        // Build 3D points from waypoints
        const waypoints3D = rawWaypoints.map(wp => {
          const y = typeof terrainHeight === 'function' ? terrainHeight(wp.x, wp.y, seed) : 0;
          return new THREE.Vector3(wp.x, y + 0.18, wp.y);
        });

        // Spline interpolation for smooth curve
        const curve = new THREE.CatmullRomCurve3(waypoints3D, false, 'centripetal', 0.4);
        const totalDist = waypoints3D.reduce((acc, pt, i) => i === 0 ? 0 : acc + pt.distanceTo(waypoints3D[i-1]), 0);
        const sampleCount = Math.max(16, Math.ceil(totalDist / 0.55)); // Point every ~55cm
        const sampledPoints = curve.getPoints(sampleCount);

        // Ground each sampled point accurately to terrain surface
        pathPoints = sampledPoints.map(pt => {
          const ty = typeof terrainHeight === 'function' ? terrainHeight(pt.x, pt.z, seed) : pt.y;
          return new THREE.Vector3(pt.x, ty + 0.16, pt.z);
        });
      }

      this.renderRoute();
      lastCalcTime = performance.now();
      return pathPoints;
    },

    /**
     * Renders or updates the visual holographic ribbon in Three.js scene.
     */
    renderRoute: function(){
      if (!window.surfaceScene) return;

      if (routeMesh){
        surfaceScene.remove(routeMesh);
        if (routeMesh.geometry) routeMesh.geometry.dispose();
        routeMesh = null;
      }

      if (!this.enabled || pathPoints.length < 2) return;

      const texture = createTrailTexture();
      const geo = buildTerrainRibbonGeometry(pathPoints, 0.52);

      const mat = new THREE.MeshBasicMaterial({
        map: texture,
        color: 0x4fd1ff,
        transparent: true,
        opacity: 0.88,
        side: THREE.DoubleSide,
        depthWrite: false,
        blending: THREE.AdditiveBlending
      });

      routeMesh = new THREE.Mesh(geo, mat);
      routeMesh.renderOrder = 999; // Ensure it renders on top of the terrain
      surfaceScene.add(routeMesh);
    },

    /**
     * Call every frame to animate the flowing chevrons toward the objective.
     */
    update: function(dt){
      animationTime += dt || 0.016;
      if (trailTexture){
        // Speed of flowing pulse toward destination
        trailTexture.offset.x -= (dt || 0.016) * 1.85;
      }
      if (routeMesh && routeMesh.material){
        // Subtle breathing glow
        routeMesh.material.opacity = 0.78 + Math.sin(animationTime * 3.5) * 0.14;
      }
    },

    /**
     * Force clear the route visual and geometry.
     */
    clear: function(){
      if (routeMesh && window.surfaceScene){
        surfaceScene.remove(routeMesh);
        if (routeMesh.geometry) routeMesh.geometry.dispose();
        routeMesh = null;
      }
      pathPoints = [];
      rawWaypoints = [];
      activeTarget = null;
    },

    /**
     * Returns an array of THREE.Vector3 points of the active path.
     */
    getPath: function(){
      return pathPoints;
    },

    /**
     * Returns the total route distance along the calculated path in meters.
     */
    getTotalDistance: function(){
      let dist = 0;
      for (let i = 1; i < pathPoints.length; i++){
        dist += pathPoints[i].distanceTo(pathPoints[i - 1]);
      }
      return dist;
    },

    /**
     * Computes the perpendicular/shortest distance from any position (default: player's surfacePos)
     * to the active navigation path ribbon.
     * Returns distance in meters.
     */
    getDistanceToPoint: function(pos){
      const p = pos || (typeof surfacePos !== 'undefined' ? surfacePos : null);
      if (!p || pathPoints.length < 2) return Infinity;

      let minDistanceSq = Infinity;
      const px = p.x;
      const pz = p.z;

      for (let i = 0; i < pathPoints.length - 1; i++){
        const a = pathPoints[i];
        const b = pathPoints[i + 1];

        const dx = b.x - a.x;
        const dz = b.z - a.z;
        const lenSq = dx * dx + dz * dz;

        let t = 0;
        if (lenSq > 0.00001){
          t = Math.max(0, Math.min(1, ((px - a.x) * dx + (pz - a.z) * dz) / lenSq));
        }

        const projX = a.x + t * dx;
        const projZ = a.z + t * dz;
        const distSq = (px - projX) * (px - projX) + (pz - projZ) * (pz - projZ);

        if (distSq < minDistanceSq){
          minDistanceSq = distSq;
        }
      }

      return Math.sqrt(minDistanceSq);
    },

    /**
     * Shortcut for measuring current player distance from the route.
     */
    getDistanceToPlayer: function(){
      return this.getDistanceToPoint(typeof surfacePos !== 'undefined' ? surfacePos : null);
    },

    /**
     * Returns detailed proximity info: closest point on path, distance, and route progress (0.0 to 1.0).
     */
    getClosestPointOnPath: function(pos){
      const p = pos || (typeof surfacePos !== 'undefined' ? surfacePos : null);
      if (!p || pathPoints.length < 2){
        return { point: null, distance: Infinity, progress: 0 };
      }

      let minDistanceSq = Infinity;
      let closestPoint = new THREE.Vector3();
      let bestSegmentIdx = 0;
      let bestT = 0;

      const px = p.x;
      const pz = p.z;

      for (let i = 0; i < pathPoints.length - 1; i++){
        const a = pathPoints[i];
        const b = pathPoints[i + 1];

        const dx = b.x - a.x;
        const dz = b.z - a.z;
        const lenSq = dx * dx + dz * dz;

        let t = 0;
        if (lenSq > 0.00001){
          t = Math.max(0, Math.min(1, ((px - a.x) * dx + (pz - a.z) * dz) / lenSq));
        }

        const projX = a.x + t * dx;
        const projY = a.y + t * (b.y - a.y);
        const projZ = a.z + t * dz;
        const distSq = (px - projX) * (px - projX) + (pz - projZ) * (pz - projZ);

        if (distSq < minDistanceSq){
          minDistanceSq = distSq;
          closestPoint.set(projX, projY, projZ);
          bestSegmentIdx = i;
          bestT = t;
        }
      }

      // Calculate progress fraction along full path
      let distSoFar = 0;
      let totalDist = 0;
      for (let i = 0; i < pathPoints.length - 1; i++){
        const segLen = pathPoints[i].distanceTo(pathPoints[i + 1]);
        if (i < bestSegmentIdx){
          distSoFar += segLen;
        } else if (i === bestSegmentIdx){
          distSoFar += segLen * bestT;
        }
        totalDist += segLen;
      }

      const progress = totalDist > 0 ? Math.min(1, Math.max(0, distSoFar / totalDist)) : 0;
      return {
        point: closestPoint,
        distance: Math.sqrt(minDistanceSq),
        progress: progress
      };
    },

    /**
     * Checks if player is within a specific deviation tolerance of the path.
     */
    isPlayerOnTrack: function(tolerance = 3.5){
      return this.getDistanceToPlayer() <= tolerance;
    }
  };

  // Expose global accessors
  window.NavigationGuide = NavigationGuide;
  window.getDistanceToNavigationPath = function(pos){
    return NavigationGuide.getDistanceToPoint(pos);
  };
})();
