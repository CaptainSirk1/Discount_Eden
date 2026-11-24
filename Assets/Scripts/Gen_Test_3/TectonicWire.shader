// Unlit constant-colour lines for the tectonics mesh wireframe.
// Pulled slightly toward the camera so edges sit on top of the coplanar surface triangles.
Shader "PlanetGen/TectonicWire"
{
    Properties
    {
        _Color ("Color", Color) = (0, 0, 0, 0.35)
    }
    SubShader
    {
        Tags { "RenderType" = "Transparent" "Queue" = "Geometry+10" }
        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
        ZTest LEqual
        Offset -1, -1
        Cull Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            fixed4 _Color;
            struct appdata { float4 vertex : POSITION; };
            struct v2f { float4 pos : SV_POSITION; };

            v2f vert (appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target { return _Color; }
            ENDCG
        }
    }
}
